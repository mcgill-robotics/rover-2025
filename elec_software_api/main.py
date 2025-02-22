import time
import serial


BAUDRATE = 9600

connected_boards = {}   # Keys are strings, values are Serial connections


def _request_name(serial_connection: serial.Serial):
    """
    Requests name from serial device and returns it. Returns None if
    name not found. Function sends 12 name requests before giving up.
    Note: Function assumes that serial_connection is open. Function
    also does not close serial_connection.

    Args:
        serial_connection (Serial): Connection to serial port.

    Returns:
        str: The name returned by the device (None if name not found).
    """
    
    for i in range(1, 13):
        serial_connection.write(("name").encode())
        time.sleep(0.01)
        serial_response = serial_connection.readline()
        serial_response = serial_response.decode()
        serial_response = serial_response.rstrip()
        
        return serial_response
    
    return None


def connect_devices() -> None:
    """
    Clears 'connected_boards' dictionary, then identifies all
    microcontrollers connected to computer via USB and places them in
    'connected_boards' (key corresponds to device name (str), value
    corresponds to connection (Serial object)).

    Args:
        None

    Returns:
        None
    """

    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "VID:PID" in port.hwid:
            serial_connection = serial.Serial(
                port=port.device,
                baudrate=BAUDRATE,
                timeout = 1)
            connection_name = _request_name(serial_connection)
            if connection_name != None:
                connected_boards[connection_name] = serial_connection
            serial_connection.close()
    
    return None


if __name__ == "__main__":
    print("Connecting devices")
    connect_devices()
    print("Connected devices")
    
    # command:value

    pass