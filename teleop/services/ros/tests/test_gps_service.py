#!/usr/bin/env python3
"""
Test script for the GPS service.
Tests the GPS data subscriber and API endpoints.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import time
import requests
import json

class GPSServiceTester(Node):
    def __init__(self):
        super().__init__('gps_service_tester')
        
        # Publisher for testing
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
        
        # Test data
        self.test_gps_data = [45.5049216, -73.56316]
        self.test_imu_data = Imu()
        self.test_imu_data.header.frame_id = 'base_link'
        self.test_imu_data.orientation.w = 1.0
        self.test_imu_data.angular_velocity.z = 0.1
        self.test_imu_data.linear_acceleration.z = 9.81
        
        self.get_logger().info('GPS Service Tester initialized')
    
    def publish_test_data(self):
        """Publish test GPS and IMU data."""
        # Publish GPS data
        gps_msg = Float64MultiArray()
        gps_msg.data = self.test_gps_data
        self.gps_publisher.publish(gps_msg)
        
        # Publish IMU data
        self.test_imu_data.header.stamp = self.get_clock().now().to_msg()
        self.imu_publisher.publish(self.test_imu_data)
        
        self.get_logger().info(f'Published test data: GPS={self.test_gps_data}')
    
    def test_api_endpoints(self):
        """Test the GPS API endpoints."""
        base_url = 'http://localhost:8082'
        
        try:
            # Test GPS coordinates endpoint
            response = requests.get(f'{base_url}/api/gps/coordinates')
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f'GPS coordinates API: {data}')
            else:
                self.get_logger().error(f'GPS coordinates API failed: {response.status_code}')
            
            # Test IMU data endpoint
            response = requests.get(f'{base_url}/api/gps/imu')
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f'IMU data API: {data}')
            else:
                self.get_logger().error(f'IMU data API failed: {response.status_code}')
            
            # Test GPS summary endpoint
            response = requests.get(f'{base_url}/api/gps/summary')
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f'GPS summary API: {data}')
            else:
                self.get_logger().error(f'GPS summary API failed: {response.status_code}')
                
        except requests.exceptions.ConnectionError:
            self.get_logger().error('Could not connect to API server. Make sure ros_manager.py is running.')
        except Exception as e:
            self.get_logger().error(f'API test error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    tester = GPSServiceTester()
    
    # Publish test data
    tester.publish_test_data()
    
    # Wait a moment for data to be processed
    time.sleep(2)
    
    # Test API endpoints
    tester.test_api_endpoints()
    
    # Cleanup
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 