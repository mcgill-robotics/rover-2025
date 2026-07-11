import serial
import math

class PanTiltGPS():

    ''' Represents the UART board with GPS and pantilt relay over a single USB CDC port.

    The board exposes one serial port that:
      - outputs GPS data as: satellites,latitude,longitude,...\\n
      - accepts pantilt commands forwarded to the servo board over UART

    Attributes
    ----------
    port: str
        The USB CDC port for both GPS output and pantilt input.
    baud_rate: int
        The baud rate of the connection.
    ser: serial.Serial
        The serial connection to the board.
    is_connected: bool
        Whether the board is connected to the computer.
    buffer: bytes
        Receive buffer for incomplete lines.
    gps_sats: float
        Number of satellites connected to the GPS.
    coords: list[float]
        The latitude and longitude [lat, lon].
    '''

    def __init__(self, port: str, baud_rate: int = 115200):
        '''
        Parameters
        ----------
        port: str
            The USB CDC port (COM? for Windows, /dev/ttyACM? for Linux).
        baud_rate: int, optional
            The baud rate of the connection. Default is 115200 bps.
        '''
        self.port: str = port
        self.baud_rate: int = baud_rate
        self.ser: serial.Serial = None
        self.is_connected: bool = False
        self.buffer: bytes = b""
        self.gps_sats: float = 0
        self.coords: list[float] = [-1.0, -1.0]

    def connect(self):
        ''' Connects to the board. Run this before using this object.

        Raises
        ------
        ConnectionError
            If it fails to connect to the board.
        '''
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_connected = True
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to board. Error: {e}")

    def _read_serial_gps(self):
        ''' Reads GPS data from serial if available. '''
        if not self.is_connected:
            raise ConnectionError("Cannot read from serial port, not connected to board.")

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except serial.SerialException:
            self.is_connected = False
            return
        if data:
            self.buffer += data
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                try:
                    self._parse_data(line.decode('utf-8').strip())
                except UnicodeDecodeError:
                    pass

    def _parse_data(self, line: str):
        ''' Parses a GPS data line from the board (satellites,lat,lon,...). '''
        data = line.split(',')
        if len(data) < 3:
            return
        try:
            self.gps_sats = float(data[0])
            self.coords[0] = float(data[1])
            self.coords[1] = float(data[2])
        except ValueError:
            pass

    def run(self):
        ''' Runs the object's main loop. Call this function in your main loop. '''
        self._read_serial_gps()

    def is_gps_connected(self) -> bool:
        ''' Returns whether the GPS has at least 3 satellite connections. '''
        return self.gps_sats >= 3

    def get_gps_satellites(self) -> float:
        ''' Gets the number of satellites connected to the GPS. '''
        return self.gps_sats

    def get_gps(self) -> list[float]:
        ''' Gets the last available GPS coordinates as [satellites, latitude, longitude]. '''
        self.print_gps_data()
        return [float(self.gps_sats), self.coords[0], self.coords[1]]

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

        print("Your GPS data:" + str(new_list) + " = " + converted)
        return
    
    def add_pan_angle(self, angle: float):
        ''' Adds an increment of angle to the pan servo.

        Parameters
        ----------
        angle : float
            The increment to add to the pan servo angle.

        Raises
        ------
        ConnectionError
            If there is no connection.
        '''
        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        try:
            self.ser.write(f"{angle},0.0\n".encode())
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to write pan angle. Error: {e}")

    def add_tilt_angle(self, angle: float):
        ''' Adds an increment of angle to the tilt servo.

        Parameters
        ----------
        angle : float
            The increment to add to the tilt servo angle.

        Raises
        ------
        ConnectionError
            If there is no connection.
        '''
        if not self.is_connected:
            raise ConnectionError("Cannot write to serial port, not connected to board.")
        try:
            self.ser.write(f"0.0,{angle}\n".encode())
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to write tilt angle. Error: {e}")


if __name__ == "__main__":
    import time
    board = PanTiltGPS("/dev/ttyACM0")  # TODO: adjust port
    try:
        board.connect()
    except ConnectionError as e:
        print(e)
        exit(1)

    board.add_pan_angle(30)
    board.add_tilt_angle(30)

    while True:
        board.run()
        print(f'GPS ON?: {board.is_gps_connected()} || GPS: {board.get_gps()}')
        time.sleep(0.05)
