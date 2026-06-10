import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

ENG_CAF_LAT = 45.5058488
ENG_CAF_LONG = -73.5761445


class MockGPSNode(Node):

    def __init__(self):
        super().__init__("mock_gps_node")
        self.gps_pusblisher = self.create_publisher(Float32MultiArray, "roverGPSData", 10)
        self.imu_pusblisher = self.create_publisher(Float32MultiArray, "roverIMUData", 10)

        timer_period = 1e-2
        self.timer = self.create_timer(timer_period, self.run)

    def run(self):
        gps_data = [1.0, ENG_CAF_LAT, ENG_CAF_LONG]
        gps_msg = Float32MultiArray()
        gps_msg.data = gps_data
        self.gps_pusblisher.publish(gps_msg)

    
if __name__ == "__main__":
    rclpy.init()
    gps_mock_node = MockGPSNode()
    rclpy.spin(gps_mock_node)
    rclpy.shutdown()