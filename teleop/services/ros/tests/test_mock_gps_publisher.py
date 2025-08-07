#!/usr/bin/env python3
"""
Mock GPS publisher for testing the GPS service.
Publishes GPS coordinates and IMU data to test the frontend integration.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import math
import time

class MockGPSPublisher(Node):
    def __init__(self):
        super().__init__('mock_gps_publisher')
        
        # Publishers
        self.gps_publisher = self.create_publisher(
            Float64MultiArray,
            '/gps_coordinates',
            10
        )
        
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu_data',
            10
        )
        
        # Timer for publishing data
        self.timer = self.create_timer(1.0, self.publish_data)
        
        # Initial position (McGill University coordinates)
        self.latitude = 45.5049216
        self.longitude = -73.56316
        self.heading = 0.0
        
        self.get_logger().info('Mock GPS Publisher initialized')
    
    def publish_data(self):
        """Publish GPS and IMU data."""
        try:
            # Publish GPS coordinates
            gps_msg = Float64MultiArray()
            gps_msg.data = [self.latitude, self.longitude]
            self.gps_publisher.publish(gps_msg)
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            
            # Set orientation (quaternion from heading)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(self.heading / 2.0)
            imu_msg.orientation.w = math.cos(self.heading / 2.0)
            
            # Set angular velocity
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.1  # Small rotation
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 9.81  # Gravity
            
            self.imu_publisher.publish(imu_msg)
            
            # Update position for next iteration (simulate movement)
            self.longitude += 0.0001  # Move east
            self.heading += 0.1  # Rotate
            
            self.get_logger().debug(f'Published GPS: {self.latitude:.6f}, {self.longitude:.6f}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    publisher = MockGPSPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 