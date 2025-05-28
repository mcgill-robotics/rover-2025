import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import pantilt_firmware as pf
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from std_msgs.msg import Float32MultiArray

class pantilt(Node):

    def __init__(self):
        super().__init__("pantilt_node")
        self.gampepad_subscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.update_pantilt, 10)
        self.pantilt_firmware = pf.PanTiltGPS("/dev/ttyACM0")
        try:
            self.pantilt_firmware.connect()
        except:
            self.get_logger().error("Failed to connect to the PanTiltGPS board. Check the connection.")
            return
        self.step_size = 5 #in degrees
        timer_period = 1e-1

        self.gps_publisher = self.create_publisher(Float32MultiArray, "roverGPSData", 10)
        self.imu_publisher = self.create_publisher(Float32MultiArray, "roverIMUData", 10)

        self.timer = self.create_timer(timer_period, self.run)

    def run(self):
        # This method calls the run() method of the firmware, and sends the IMU and GPS data to topics.
        self.pantilt_firmware.run()
        # Get IMU data and publish it
        imu_data = self.pantilt_firmware.get_imu_data()
        imu_msg = Float32MultiArray()
        for i in range(len(imu_data)):
            imu_data[i] = float(imu_data[i])
        imu_msg.data = tuple(imu_data)
        self.imu_publisher.publish(imu_msg)
        # Get GPS data and publish it
        gps_data = self.pantilt_firmware.get_gps()
        gps_msg = Float32MultiArray()
        gps_msg.data = tuple(gps_data)
        self.gps_publisher.publish(gps_msg)


    def update_pantilt(self, gamepad_input : GamePadInput) :
        # Update the angles based on the gamepad input
        tilt_change = gamepad_input.d_pad_y * self.step_size
        pan_change = gamepad_input.d_pad_x * self.step_size
        # Control the servos
        try:
            self.pantilt_firmware.add_pan_angle(pan_change)
            self.pantilt_firmware.add_tilt_angle(tilt_change)
        except ConnectionError as e:
            self.get_logger().error(f"Failed to update pan/tilt angles: {e}")
            return
        

def main(args=None):
    rclpy.init(args=args)
    pantilt_node = pantilt()
    rclpy.spin(pantilt_node)

if __name__ == "__main__":
    main()