import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import base_station_antenna_firmware as bs_antenna_f
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import argparse

class basestation_antenna_node(Node):

    def __init__(self, serial_port: str = "/dev/ttyACM0", 
                 basestation_antenna_coords: tuple[float] = None,
                 azimuth_angle: int = None,
                 servo_pos: float = None,
                 auto_config: tuple[bool] = None):
        super().__init__("basestation_antenna_node")
        
        self.rover_GPS_subscriber = self.create_subscription(Float32MultiArray, "roverGPSData", self.update_antenna, 10)            
        try:
            self.basestation_antenna_firmware = bs_antenna_f.SerialAPI(port=serial_port)
        except:
            self.get_logger().error("Failed to connect to the Basestation Antenna board. Check the connection.")
            # return
        
        if None not in basestation_antenna_coords:
            self.basestation_antenna_firmware.send_basegps_coordinates(*basestation_antenna_coords)
        if azimuth_angle:
            assert -180 <= azimuth_angle <= 180
            self.basestation_antenna_firmware.send_azimuth_angle(azimuth_angle)
        if servo_pos:
            assert 0 <= servo_pos <= 180
            self.basestation_antenna_firmware.send_servo_position(servo_pos)

        if auto_config:
            auto_commands = { 0: self.basestation_antenna_firmware.send_basegps_auto,
                              1: self.basestation_antenna_firmware.send_azimuth_auto,
                              2: self.basestation_antenna_firmware.send_servo_auto
                            }
            for i, config in enumerate(auto_config):
                if config:
                    auto_commands[i]()

    def update_antenna(self, roverGPSData : Float32MultiArray) :
        latitude, longtitude = roverGPSData.data
        try:
            self.basestation_antenna_firmware.send_rover_coordinates(lat=latitude, lon=longtitude)
        except ConnectionError as e:
            self.get_logger().error(f"Failed to pass rover coordinates: {e}")
        

def main(args=None):
    # Parse command line arguments before rclpy.init()
    parser = argparse.ArgumentParser(description="Basestation Antenna Node")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port to connect to antenna")
    parser.add_argument("--lat",  type=float, default=None, help="Latitude of Antenna (Float)")
    parser.add_argument("--lon",  type=float, default=None, help="Longtitude of Antenna (Float)")
    parser.add_argument("--azimuth"    ,  type=float, default=None, help="Azimuth Agnle of Antenna (int)")
    parser.add_argument("--servo"      ,  type=float, default=None, help="Servo position of Antenna (float)")
    parser.add_argument("--auto_coords",  type=bool, default=False, help="Set auto for GPS coords")
    parser.add_argument("--auto_azimuth", type=bool, default=False, help="Set auto for azimuth angle")
    parser.add_argument("--auto_servo",   type=bool, default=False, help="Set auto for servo position")
    known_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    basestat_antenna_node = basestation_antenna_node(serial_port=known_args.port, 
                                                     basestation_antenna_coords = (known_args.lat, known_args.lon),
                                                     azimuth_angle = known_args.azimuth,
                                                     servo_pos = known_args.servo,
                                                     auto_config = (known_args.auto_coords, known_args.auto_azimuth, known_args.auto_servo))
    rclpy.spin(basestat_antenna_node)

if __name__ == "__main__":
    main()