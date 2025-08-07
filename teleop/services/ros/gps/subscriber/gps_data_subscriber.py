"""
ROS2 subscriber node for collecting GPS and IMU data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import sys
import os

# Add utils to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'utils'))
from ros_bridge import DataCollector

import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)


class GPSDataSubscriber(Node):
    """
    ROS2 node that subscribes to GPS and IMU topics and collects data for web services.
    """
    
    def __init__(self, data_collector: DataCollector):
        super().__init__('gps_data_subscriber')
        
        self.data_collector = data_collector
        
        # Subscribers
        self.gps_subscriber = self.create_subscription(
            Float64MultiArray,
            '/gps_coordinates',  # Topic for latitude and longitude
            self.gps_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu_data',  # Topic for IMU data
            self.imu_callback,
            10
        )
        
        # Data tracking
        self.last_gps_time = 0
        self.last_imu_time = 0
        
        self.get_logger().info('GPS Data Subscriber initialized')
    
    def gps_callback(self, msg: Float64MultiArray):
        """Handle GPS coordinate messages."""
        try:
            # Expecting [latitude, longitude] format
            if len(msg.data) >= 2:
                gps_data = {
                    'coordinates': {
                        'latitude': float(msg.data[0]),
                        'longitude': float(msg.data[1])
                    },
                    'timestamp': time.time()
                }
                
                self.data_collector.update_data('gps_coordinates', gps_data)
                self.last_gps_time = time.time()
                
                self.get_logger().debug(f'Updated GPS coordinates: {gps_data}')
            else:
                self.get_logger().warning(f'GPS message has insufficient data: {len(msg.data)} values')
                
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {e}')
    
    def imu_callback(self, msg: Imu):
        """Handle IMU messages."""
        try:
            # Extract relevant IMU data
            imu_data = {
                'orientation': {
                    'x': float(msg.orientation.x),
                    'y': float(msg.orientation.y),
                    'z': float(msg.orientation.z),
                    'w': float(msg.orientation.w)
                },
                'angular_velocity': {
                    'x': float(msg.angular_velocity.x),
                    'y': float(msg.angular_velocity.y),
                    'z': float(msg.angular_velocity.z)
                },
                'linear_acceleration': {
                    'x': float(msg.linear_acceleration.x),
                    'y': float(msg.linear_acceleration.y),
                    'z': float(msg.linear_acceleration.z)
                },
                'timestamp': time.time()
            }
            
            self.data_collector.update_data('imu_data', imu_data)
            self.last_imu_time = time.time()
            
            self.get_logger().debug(f'Updated IMU data: {imu_data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')
    
    def get_connection_status(self) -> dict:
        """Get connection status for all data streams."""
        current_time = time.time()
        timeout_threshold = 5.0  # 5 seconds
        
        return {
            'gps_connected': (current_time - self.last_gps_time) < timeout_threshold,
            'imu_connected': (current_time - self.last_imu_time) < timeout_threshold,
            'last_update': {
                'gps': self.last_gps_time,
                'imu': self.last_imu_time
            }
        }
    
    def get_summary_data(self) -> dict:
        """Get a summary of all GPS and IMU data for quick access."""
        gps_data = self.data_collector.get_data('gps_coordinates')
        imu_data = self.data_collector.get_data('imu_data')
        
        summary = {
            'connection_status': self.get_connection_status(),
            'data_available': {
                'gps': gps_data is not None,
                'imu': imu_data is not None
            }
        }
        
        # Add latest data if available
        if gps_data:
            summary['latest_gps'] = gps_data['data']
        if imu_data:
            summary['latest_imu'] = imu_data['data']
            
        return summary


def create_gps_subscriber(data_collector: DataCollector) -> GPSDataSubscriber:
    """Factory function to create a GPS data subscriber."""
    return GPSDataSubscriber(data_collector) 