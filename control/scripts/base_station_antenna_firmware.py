import serial
import time

class SerialAPI:
    def __init__(self, port='COM3', baud=9600, timeout=1):
        """Open the serial port and wait for the Teensy to be ready."""
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)
        
    # Method to send commands to the Teensy
    def send_cmd(self, cmd: str, payload: str):
        """Send a command like 'AZIMUTH:123' or 'BASEGPS:45.5,-73.5'."""
        line = f"{cmd}:{payload}\n"
        self.ser.write(line.encode('utf-8'))
        print("Sent ▶", line.strip())
        # give Teensy time to reply
        time.sleep(2)
        if self.ser.in_waiting:
            print("Recv ◀", self.ser.readline().decode().strip())

    # For sending rover GPS coordinates
    def send_rover_coordinates(self, lat: float, lon: float):
        send_cmd = self.send_cmd
        send_cmd("ROVERGPS", f"{lat:.12f},{lon:.12f}")

    # For sending base station GPS coordinates
    def send_basegps_coordinates(self, lat: float, lon: float):
        send_cmd = self.send_cmd
        send_cmd("BASEGPS", f"{lat:.12f},{lon:.12f}")

    # For resetting the base station GPS to automatic mode
    def send_basegps_auto(self):
        self.send_cmd("BASEGPS_AUTO", "")
        
    # For sending azimuth angle
    def send_azimuth_angle(self, angle: int):
        self.send_cmd("AZIMUTH", str(angle))

    # For resetting the azimuth to automatic mode
    def send_azimuth_auto(self):
        self.send_cmd("AZIMUTH_AUTO", "")

    # For sending servo position
    def send_servo_position(self, angle: float):
        self.send_cmd("SERVO", f"{angle:.2f}")

    # For resetting the servo to automatic mode
    def send_servo_auto(self):
        self.send_cmd("SERVO_AUTO", "")

# This is a simple test script to demonstrate the API (just for testing purposes)
if __name__ == "__main__":
    api = SerialAPI()
    api.send_basegps_coordinates(45.696969696969, -73.696969696969)
    time.sleep(2)
    api.send_azimuth_angle(135)
    time.sleep(2)
    api.send_servo_position(90)
    time.sleep(2)
    api.send_rover_coordinates(45.696969696969, -73.696969696969)
    time.sleep(2)

    # back to automatic
    api.send_basegps_auto()
    time.sleep(2)
    api.send_azimuth_auto()
    time.sleep(2)
    api.send_servo_auto()
    time.sleep(2)
    