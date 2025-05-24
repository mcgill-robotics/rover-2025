import serial

class PanTiltGPS():

    ''' This class represents the pan tilt/gps/servo board.
    
    Attributes
    ----------
    port: str
        The port to connect to.
    baud_rate: int
        The baud rate of the connection
    ser: serial.Serial
        The serial object from pyserial
    is_connected: bool
        Whether the board is connected to the computer
    buffer: bytes
        The buffer of bytes received from the serial port
    has_gps: bool
        Whether the board has at least one connected satellites
    coords: list[float]
        The latitude and longitude
    imu: list[float]
        The imu data received from the board

    Notes
    ------
    The board has been limited to a 100Hz loop. To increase this rate, it needs to be changed on the firmware.
    '''

    def __init__(self, port: str, baud_rate: int = 115200):
        ''' The initializer for the PanTiltGPS object.
        
        Parameters
        ----------
        port: str
            The port of the computer that the board is connected to (COM? for Windows, /dev/ttyACM? for Linux)
        baud_rate: int, optional
            The baud rate of the connection. Default is 115200 bps
        '''
        self.port: str = port
        self.baud_rate: int = baud_rate
        self.ser: serial.Serial = None
        self.is_connected: bool = False
        self.buffer : bytes = b""
        self.has_gps: bool = False
        self.coords: list[float] = [-1.0, -1.0]
        self.imu: list[float] = [0, 0, 0, 0, 0, 0]

    def connect(self):
        ''' Connects to the Pan Tilt/GPS board. Run this before using this object.

        Raises
        ------
        ConnectionError:
            If if fails to connect to the board.
        '''
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_connected = True
        except serial.SerialException as e:
            raise ConnectionError(f"Failed to connect to Pan Tilt Board. Error: {e}")
        

    def _read_serial(self):
        ''' Reads from serial if it is available
        
        Raises
        ------
        ConnectionError 
            If there is no connection, and reading cannot be done.
        '''
        if self.is_connected is False:
            raise ConnectionError("Cannot read from serial port, not connected to board.")
            
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.buffer += data
            while True:
                if b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    line = line.decode('utf-8').strip()
                    self._parse_data(line)
                else:
                    break
                
    def _parse_data(self, line: str):
        '''  Parses the data for a received line from the board.
        '''
        data = line.split(',')
        if len(data) != 9:
            return
        self.has_gps = True if float(data[0]) > 0 else False
        self.coords[0] = float(data[1])
        self.coords[1] = float(data[2])
        for i in range(3,9):
            self.imu[i-3] = float(data[i])

    def run(self):
        ''' Runs the object's main loop. Call this function in your main loop.
        '''
        self._read_serial()

    def is_gps_connected(self) -> bool:
        ''' Returns whether the GPS has at least one satellite connection
        
        Returns
        -------
        bool
            True if the gps is connected, False if not
        '''
        return self.has_gps

    def get_gps(self) -> list[float]:
        ''' Gets the last available gps coordinates (latitude, longitude)
        
        Returns
        -------
        list[float]
            The list of latitude and longitude coordinates
        '''
        return self.coords
    
    def get_imu_data(self) -> list[float]:
        ''' Gets the last available imu data.
            The format is (Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z)
        
        Returns
        -------
        list[float]
            The list of acceleration and gyro values obtained. See description for format.
        '''
        return self.imu

    def add_pan_angle(self, angle: float):
        ''' Adds an increment of angle to the pan servo.
        
        Parameters
        ----------
        angle : float
            The increment to add to the angle of the pan servo

        Raises
        ------
        ConnectionError 
            If there is no connection, and the servo cannot be controlled
        '''
        if self.is_connected is False:
            raise ConnectionError("Cannot write from serial port, not connected to board.")
        message = (f"{angle},0.0\n").encode()
        self.ser.write(message)

    def add_tilt_angle(self, angle: float):
        ''' Adds an increment of angle to the tilt servo.
        
        Parameters
        ----------
        angle : float
            The increment to add to the angle of the tilt servo

        Raises
        ------
        ConnectionError 
            If there is no connection, and the servo cannot be controlled
        '''
        if self.is_connected is False:
            raise ConnectionError("Cannot write from serial port, not connected to board.")
        message = (f"0.0,{angle}\n").encode()
        self.ser.write(message)



if __name__ == "__main__":
    # Test script
    import time
    board = PanTiltGPS("COM4")
    try:
        board.connect()
    except ConnectionError as e:
        print(e)
        exit(1)

    while True:
        # print data, no servo for now
        board.run()
        is_gps = board.is_gps_connected()
        gps = board.get_gps()
        imu = board.get_imu_data()
        print(f'GPS ON?: {is_gps} || GPS: {gps} || IMU: {imu}')
        time.sleep(0.05)