"""
ROS2 subscriber node for collecting drive system data from firmware nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from std_msgs.msg import Float32MultiArray
from msg_srv_interface.msg import DriveMotorDiagnostic
from msg_srv_interface.srv import DriveMotorStatus
import sys
import os

# Add utils to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'utils'))
from ros_bridge import DataCollector, create_motor_diagnostic_dict, create_motor_status_dict

import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)


class DriveDataSubscriber(Node):
    """
    ROS2 node that subscribes to drive system topics and collects data for web services.
    """
    
    def __init__(self, data_collector: DataCollector):
        super().__init__('drive_data_subscriber')
        
        self.data_collector = data_collector
        
        # Subscribers
        self.motor_diagnostic_subscriber = self.create_subscription(
            DriveMotorDiagnostic,
            'drive_motors_info',
            self.motor_diagnostic_callback,
            10
        )
        
        self.motor_speeds_subscriber = self.create_subscription(
            Float32MultiArray,
            'drive_speeds_info',
            self.motor_speeds_callback,
            10
        )
        
        # Service client for motor status
        self.motor_status_client: Client = self.create_client(
            DriveMotorStatus,
            'drive_motors_status'
        )
        
        # Timer for periodic status requests
        self.status_timer = self.create_timer(2.0, self.request_motor_status)
        
        # Data tracking
        self.last_diagnostic_time = 0
        self.last_speeds_time = 0
        self.last_status_time = 0
        
        self.get_logger().info('Drive Data Subscriber initialized')
    
    def motor_diagnostic_callback(self, msg: DriveMotorDiagnostic):
        """Handle motor diagnostic messages."""
        try:
            diagnostic_data = create_motor_diagnostic_dict(msg)
            self.data_collector.update_data('drive_diagnostics', diagnostic_data)
            self.last_diagnostic_time = time.time()
            
            self.get_logger().debug(f'Updated motor diagnostics: {diagnostic_data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing motor diagnostic: {e}')
    
    def motor_speeds_callback(self, msg: Float32MultiArray):
        """Handle motor speed messages."""
        try:
            # Convert speeds array to structured format
            # Order: [RF, RB, LB, LF] based on firmware node
            speeds_data = {
                'speeds': {
                    'RF': float(msg.data[0]) if len(msg.data) > 0 else 0.0,
                    'RB': float(msg.data[1]) if len(msg.data) > 1 else 0.0,
                    'LB': float(msg.data[2]) if len(msg.data) > 2 else 0.0,
                    'LF': float(msg.data[3]) if len(msg.data) > 3 else 0.0,
                },
                'timestamp': time.time()
            }
            
            self.data_collector.update_data('drive_speeds', speeds_data)
            self.last_speeds_time = time.time()
            
            self.get_logger().debug(f'Updated motor speeds: {speeds_data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing motor speeds: {e}')
    
    def request_motor_status(self):
        """Periodically request motor status via service call."""
        if not self.motor_status_client.service_is_ready():
            self.get_logger().debug('Motor status service not ready')
            return
        
        try:
            request = DriveMotorStatus.Request()
            future = self.motor_status_client.call_async(request)
            future.add_done_callback(self.motor_status_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error requesting motor status: {e}')
    
    def motor_status_callback(self, future):
        """Handle motor status service response."""
        try:
            response = future.result()
            status_data = create_motor_status_dict(response)
            self.data_collector.update_data('drive_status', status_data)
            self.last_status_time = time.time()
            
            self.get_logger().debug(f'Updated motor status: {status_data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing motor status response: {e}')
    
    def get_connection_status(self) -> dict:
        """Get connection status for all data streams."""
        current_time = time.time()
        timeout_threshold = 5.0  # 5 seconds
        
        return {
            'diagnostics_connected': (current_time - self.last_diagnostic_time) < timeout_threshold,
            'speeds_connected': (current_time - self.last_speeds_time) < timeout_threshold,
            'status_connected': (current_time - self.last_status_time) < timeout_threshold,
            'service_available': self.motor_status_client.service_is_ready(),
            'last_update': {
                'diagnostics': self.last_diagnostic_time,
                'speeds': self.last_speeds_time,
                'status': self.last_status_time
            }
        }
    
    def get_summary_data(self) -> dict:
        """Get a summary of all drive data for quick access."""
        diagnostics = self.data_collector.get_data('drive_diagnostics')
        speeds = self.data_collector.get_data('drive_speeds')
        status = self.data_collector.get_data('drive_status')
        
        summary = {
            'connection_status': self.get_connection_status(),
            'data_available': {
                'diagnostics': diagnostics is not None,
                'speeds': speeds is not None,
                'status': status is not None
            }
        }
        
        # Add latest data if available
        if diagnostics:
            summary['latest_diagnostics'] = diagnostics['data']
        if speeds:
            summary['latest_speeds'] = speeds['data']
        if status:
            summary['latest_status'] = status['data']
            
        return summary


def create_drive_subscriber(data_collector: DataCollector) -> DriveDataSubscriber:
    """Factory function to create a drive data subscriber."""
    return DriveDataSubscriber(data_collector)
