import serial
import sys
import time
import math
import datetime

DELIMITER = 0x00
PANTILT_SEND_INTERVAL_S = 0.02 # 20 ms, 50 Hz rate
GPS_FILE = "PLACEHOLDER" # Todo


def cobs_encode(data: bytes) -> bytes:
    """
    Encodes data into a COBS frame.

    Parameters
    ----------
    data : bytes
        Raw bytes to encode.

    Returns
    -------
    bytes
        COBS-encoded frame, without a trailing delimiter.
    """

    output = bytearray(b'\x00')
    code_pos = 0
    code = 1
    for byte in data:
        if byte == DELIMITER:
            output[code_pos] = code
            code_pos = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_pos] = code
                code_pos = len(output)
                output.append(0)
                code = 1
    output[code_pos] = code
    return bytes(output)


def cobs_decode(frame: bytes) -> bytes:
    """
    Decodes one COBS frame.

    Parameters
    ----------
    frame : bytes
        Encoded COBS frame.

    Returns
    -------
    bytes
        Decoded payload.

    Raises
    ------
    ValueError
        If the frame is malformed.
    """

    output = bytearray()
    i, n = 0, len(frame)
    while i < n:
        code = frame[i]
        if code == 0 or i + code > n:
            raise ValueError("invalid COBS frame")
        i += 1
        output += frame[i:i + code - 1]
        i += code - 1
        if code < 0xFF and i < n:
            output.append(DELIMITER)
    return bytes(output)


class PanTiltGPS:
    """
    UART board with GPS and pantilt relay over a single USB CDC port.

    Attributes
    ----------
    port : str
        USB CDC port for GPS output and pantilt input.
    baud_rate : int
        Baud rate of the connection.
    ser : serial.Serial
        Serial connection to the board.
    is_connected : bool
        Whether the board is connected.
    buffer : bytes
        Receive buffer for an incomplete COBS frame.
    gps_sats : float
        Number of GPS satellites.
    coords : list[float]
        Latitude and longitude [lat, lon].
    heading : float
        Heading of motion in degrees.
    pan_angle : float
        Reported pan angle.
    tilt_angle : float
        Reported tilt angle.
    combine_pantilt : bool
        If False, send pan/tilt as separate frames (2x throughput, for drop testing).
    terminal_rx : bytearray
        Raw bytes received from the secondary UART.
    """

    def __init__(self, port: str, baud_rate: int = 115200, combine_pantilt: bool = True):
        """
        Parameters
        ----------
        port : str
            USB CDC port (COM? on Windows, /dev/ttyACM? on Linux).
        baud_rate : int, optional
            Baud rate of the connection. Default 115200 bps.
        combine_pantilt : bool, optional
            Default True (one combined frame). False sends pan/tilt as
            separate frames, for throughput testing.
        """
        self.last_saved_gps = None

        self.port: str = port
        self.baud_rate: int = baud_rate
        self.combine_pantilt: bool = combine_pantilt
        self.ser: serial.Serial = None
        self.is_connected: bool = False
        self.buffer: bytes = b""

        # GPS data
        self.gps_sats: float = 0
        self.coords: list[float] = [-1.0, -1.0]
        self.heading: float = 0.0

        # Pantilt data
        self.pan_angle: float = 0.0
        self.tilt_angle: float = 0.0

        # Combined deltas from pantilt commands
        self._pending_pan: float = 0.0
        self._pending_tilt: float = 0.0
        self._last_pantilt_send: float = time.monotonic()

        # Diagnostic data
        self.gps1_valid_frames: int = 0
        self.gps1_error_frames: int = 0
        self.pantilt_tx_dropped: int = 0
        self.terminal_tx_dropped: int = 0
        self.usb_tx_dropped: int = 0
        self.uart_errors: int = 0

        self.gps_startups = 0
        self.pantilt_startups = 0

        # Terminal functionality
        self.terminal_rx: bytearray = bytearray()

    def connect(self):
        """
        Opens the serial connection. Call before anything else.

        Raises
        ------
        ConnectionError
            If the connection fails.
        """

        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_connected = True
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to board. Error: {e}")

    def _read_serial_gps(self):
        """
        Reads and parses GPS/board data from serial, if available.

        Raises
        ------
        ConnectionError
            If not connected to the board.
        """

        if not self.is_connected:
            raise ConnectionError("Cannot read from serial port, not connected to board.")

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException:
            self.is_connected = False
            return
        if not data:
            return
        self.buffer += data
        while bytes([DELIMITER]) in self.buffer:
            frame, self.buffer = self.buffer.split(bytes([DELIMITER]), 1)
            if not frame:
                continue
            try:
                payload = cobs_decode(frame)
            except ValueError:
                continue
            if payload:
                self.parse_frame(payload[0:1], payload[1:])

    def parse_frame(self, msg_type: bytes, payload: bytes):
        """
        Parses a decoded [type][payload] frame from the board.

        Parameters
        ----------
        msg_type : bytes
            Single-byte frame type identifier.
        payload : bytes
            Frame payload.
        """

        text = payload.decode('utf-8', errors='replace')

        if msg_type == b"g":
            fields = text.split(',')
            if len(fields) < 4:
                return
            try:
                self.gps_sats = float(fields[0])
                self.coords[0] = float(fields[1])
                self.coords[1] = float(fields[2])
                self.heading = float(fields[3])
            except ValueError:
                pass

        elif msg_type == b"p":
            fields = text.split(',')
            if len(fields) < 2:
                return
            try:
                self.pan_angle = float(fields[0])
                self.tilt_angle = float(fields[1])
            except ValueError:
                pass

        elif msg_type == b"t":
            self.terminal_rx += payload

        elif msg_type == b"d":
            fields = text.split(',')
            if len(fields) < 2:
                return
            try:
                self.gps1_valid_frames = int(fields[0])
                self.gps1_error_frames = int(fields[1])
                if len(fields) >= 6:
                    self.pantilt_tx_dropped = int(fields[2])
                    self.terminal_tx_dropped = int(fields[3])
                    self.usb_tx_dropped = int(fields[4])
                    self.uart_errors = int(fields[5])
            except ValueError:
                pass
        elif msg_type == b"s":
            if text.strip() == "Pantilt Ready":
                self.pantilt_startups+=1
            elif text.strip() == "GPS Ready":
                self.gps_startups+=1

    def send_frame(self, msg_type: bytes, payload: bytes):
        """
        Encodes and writes a [type][payload] COBS frame to the board.

        Parameters
        ----------
        msg_type : bytes
            Single-byte frame type identifier.
        payload : bytes
            Frame payload.

        Raises
        ------
        ConnectionError
            If the write fails.
        """

        try:
            self.ser.write(cobs_encode(msg_type + payload) + bytes([DELIMITER]))
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to write to board. Error: {e}")

    def run(self):
        """Runs the object's main loop. Call this function in your main loop."""

        self._read_serial_gps()
        if self.combine_pantilt:
            self._flush_pantilt()

    def _flush_pantilt(self):
        """
        Sends queued pan/tilt deltas as one combined frame, rate-limited to PANTILT_SEND_INTERVAL_S.

        Raises
        ------
        ConnectionError
            Write fails.
        """

        if self._pending_pan == 0.0 and self._pending_tilt == 0.0:
            return
        now = time.monotonic()
        if now - self._last_pantilt_send < PANTILT_SEND_INTERVAL_S:
            return
        self.send_frame(b"p", f"{self._pending_pan},{self._pending_tilt}".encode())
        self._pending_pan = 0.0
        self._pending_tilt = 0.0
        self._last_pantilt_send = now

    def is_gps_connected(self) -> bool:
        """
        Returns
        -------
        bool
            Whether the GPS has at least 3 satellite connections.
        """

        return self.gps_sats >= 3

    def get_gps_satellites(self) -> float:
        """
        Returns
        -------
        float
            Number of satellites connected to the GPS.
        """

        return self.gps_sats

    def get_gps(self) -> list[float]:
        """
        Returns
        -------
        list[float]
            Last available GPS reading as [satellites, latitude, longitude, heading].
        """

        self.print_gps_data()

        return [float(self.gps_sats), self.coords[0], self.coords[1], self.heading]

    def print_gps_data(self) -> None:
        new_list = [float(self.gps_sats), self.coords[0], self.coords[1]]

        if self.coords[0]>0:
            lat_dir = "N"
        else:
            lat_dir = "S"

        
        if self.coords[1]>0:
            long_dir = "E"
        else:
            long_dir = "W"
            
        lat_deg = math.floor(abs(self.coords[0]))
        lat_min = math.floor((abs(self.coords[0])-lat_deg)*60)
        lat_sec = (((abs(self.coords[0])-lat_deg)*60)-lat_min)*60
        long_deg = math.floor(abs(self.coords[1]))
        long_min = math.floor((abs(self.coords[1])-long_deg)*60)
        long_sec = (((abs(self.coords[1])-long_deg)*60)-long_min)*60
        converted = f"{lat_deg}°{lat_min}\'{lat_sec}\"{lat_dir} {long_deg}°{long_min}\'{long_sec}\"{long_dir}"

        cur_time = datetime.datetime.now()
        if not self.last_saved_gps or cur_time - self.last_saved_gps >= datetime.timedelta(seconds=5):
            with open(GPS_FILE, "+a") as f:
                f.write(converted)
            self.last_saved_gps = cur_time

        print("Your GPS data:" + str(new_list) + " = " + converted)

    def get_pantilt(self) -> list[float]:
        """
        Returns
        -------
        list[float]
            Last available pantilt angles as [pan angle, tilt angle].
        """

        return [self.pan_angle, self.tilt_angle]

    def add_pan_angle(self, angle: float):
        """
        Queues a pan angle increment (combine_pantilt True), or sends it immediately as its own frame (False).

        Parameters
        ----------
        angle : float
            Increment to add to the pan angle.

        Raises
        ------
        ConnectionError
            Not connected, or write fails.
        """

        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        if self.combine_pantilt:
            self._pending_pan += angle
        else:
            self.send_frame(b"p", f"{angle},0".encode())

    def add_tilt_angle(self, angle: float):
        """
        Queues a tilt angle increment (combine_pantilt True), or sends it immediately as its own frame (False).

        Parameters
        ----------
        angle : float
            Increment to add to the tilt angle.

        Raises
        ------
        ConnectionError
            Not connected, or write fails.
        """

        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        if self.combine_pantilt:
            self._pending_tilt += angle
        else:
            self.send_frame(b"p", f"0,{angle}".encode())

    def write_terminal(self, data: bytes):
        """
        Sends raw bytes to the secondary UART.

        Parameters
        ----------
        data : bytes
            Bytes to forward. Max 255 bytes per call; split larger payloads.

        Raises
        ------
        ConnectionError
            If there is no connection.
        ValueError
            If `data` is longer than 255 bytes.
        """

        if len(data) > 255:
            raise ValueError(f"Terminal frame too large ({len(data)} bytes, max 255); split into multiple calls.")
        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        self.send_frame(b"t", data)

    def read_terminal(self) -> bytes:
        """
        Returns and clears any raw bytes received from the terminal device since the last call.

        Returns
        -------
        bytes
            Bytes received from the terminal device since the last call.
        """

        data = bytes(self.terminal_rx)
        self.terminal_rx.clear()
        return data

    def get_gps_diag(self):
        """
        Returns
        -------
        list[int]
            [valid_frames, error_frames] received by the GPS.
        """

        return [self.gps1_valid_frames, self.gps1_error_frames]

    def get_drop_counts(self):
        """
        Returns
        -------
        list[int]
            [pantilt_tx_dropped, terminal_tx_dropped, usb_tx_dropped, uart_errors].
        """

        return [self.pantilt_tx_dropped, self.terminal_tx_dropped, self.usb_tx_dropped, self.uart_errors]

    def get_startup_count(self):
        """
        Returns
        -------
        tuple[int, int]
            (gps_startups, pantilt_startups) seen since connecting.
        """

        return (self.gps_startups, self.pantilt_startups)

if __name__ == "__main__":
    import time
    board = PanTiltGPS("/dev/ttyACM0", 115200, False)
    try:
        board.connect()
    except ConnectionError as e:
        print(e)
        sys.exit(1)
    dir_pan = 1
    dir_tilt = 1
    while True:
        board.run()

        print(f"GPS lock: {board.is_gps_connected()}, GPS: {board.get_gps()}")
        pan_angle, tilt_angle = board.get_pantilt()
        print(f"Pantilt: {(pan_angle, tilt_angle)}")
        print(f"Diagnostic: {board.get_gps_diag()}")
        print(f"Drops: {board.get_drop_counts()}")
        print(f"Startup count: {board.get_startup_count()}")

        if pan_angle == 360:
            dir_pan = -1
        elif pan_angle == 0:
            dir_pan = 1
        if tilt_angle == 270:
            dir_tilt = -1
        elif tilt_angle == 0:
            dir_tilt = 1

        board.add_tilt_angle(5*dir_tilt)
        board.add_pan_angle(5*dir_pan);
        time.sleep(0.05)