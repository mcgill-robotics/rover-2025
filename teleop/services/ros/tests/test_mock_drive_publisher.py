#!/usr/bin/env python3

"""
Mock Drive Data Publisher for Testing
Publishes sample drive motor diagnostics and status data to ROS topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import random
import sys
import os
import math

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Try to import custom messages (may not be available in all environments)
try:
    from msg_interface.msg import DriveMotorDiagnostic, DriveMotorStatus
    CUSTOM_MESSAGES_AVAILABLE = True
except ImportError:
    print("Warning: Custom ROS messages not available. Using standard messages.")
    CUSTOM_MESSAGES_AVAILABLE = False


class MockDrivePublisher(Node):
    """
    Mock publisher for drive motor diagnostics and status data.
    """
    
    def __init__(self):
        super().__init__('mock_drive_publisher')
        
        # Publishers
        if CUSTOM_MESSAGES_AVAILABLE:
            self.motor_diagnostics_pub = self.create_publisher(
                DriveMotorDiagnostic,
                '/drive/motor_diagnostics',
                10
            )
            
            self.drive_status_pub = self.create_publisher(
                DriveMotorStatus,
                '/drive/status',
                10
            )
        else:
            # Fallback to standard messages
            self.motor_diagnostics_pub = self.create_publisher(
                Float64MultiArray,
                '/drive/motor_diagnostics',
                10
            )
            
            self.drive_status_pub = self.create_publisher(
                Float64MultiArray,
                '/drive/status',
                10
            )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drive/cmd_vel',
            10
        )
        
        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz
        
        # Mock data state
        self.motor_data = {
            'left_front': {'voltage': 12.0, 'current': 2.5, 'temperature': 45.0, 'status': 'OK'},
            'right_front': {'voltage': 12.1, 'current': 2.4, 'temperature': 44.0, 'status': 'OK'},
            'left_rear': {'voltage': 11.9, 'current': 2.6, 'temperature': 46.0, 'status': 'OK'},
            'right_rear': {'voltage': 12.0, 'current': 2.5, 'temperature': 45.5, 'status': 'OK'}
        }
        
        self.drive_status = {
            'overall_status': 'OPERATIONAL',
            'battery_level': 85.0,
            'system_voltage': 12.0,
            'total_current': 10.0,
            'average_temperature': 45.0
        }
        
        self.get_logger().info('Mock Drive Publisher started')
    
    def publish_data(self):
        """Publish mock drive data."""
        try:
            # Update mock data with some variation
            self.update_mock_data()
            
            # Publish motor diagnostics
            if CUSTOM_MESSAGES_AVAILABLE:
                self.publish_motor_diagnostics_custom()
                self.publish_drive_status_custom()
            else:
                self.publish_motor_diagnostics_standard()
                self.publish_drive_status_standard()
            
            # Publish command velocity (simulate some movement)
            self.publish_cmd_vel()
            
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {e}')
    
    def update_mock_data(self):
        """Update mock data with realistic variations."""
        # Update motor data
        for motor in self.motor_data.values():
            motor['voltage'] += random.uniform(-0.1, 0.1)
            motor['voltage'] = max(10.0, min(14.0, motor['voltage']))
            
            motor['current'] += random.uniform(-0.2, 0.2)
            motor['current'] = max(0.0, min(5.0, motor['current']))
            
            motor['temperature'] += random.uniform(-1.0, 1.0)
            motor['temperature'] = max(20.0, min(80.0, motor['temperature']))
            
            # Occasionally set status to warning
            if motor['temperature'] > 70.0:
                motor['status'] = 'WARNING'
            elif motor['temperature'] > 60.0:
                motor['status'] = 'CAUTION'
            else:
                motor['status'] = 'OK'
        
        # Update drive status
        self.drive_status['battery_level'] += random.uniform(-0.5, 0.5)
        self.drive_status['battery_level'] = max(0.0, min(100.0, self.drive_status['battery_level']))
        
        self.drive_status['system_voltage'] = sum(m['voltage'] for m in self.motor_data.values()) / 4
        self.drive_status['total_current'] = sum(m['current'] for m in self.motor_data.values())
        self.drive_status['average_temperature'] = sum(m['temperature'] for m in self.motor_data.values()) / 4
        
        # Update overall status based on conditions
        if self.drive_status['battery_level'] < 20.0:
            self.drive_status['overall_status'] = 'LOW_BATTERY'
        elif any(m['status'] != 'OK' for m in self.motor_data.values()):
            self.drive_status['overall_status'] = 'WARNING'
        else:
            self.drive_status['overall_status'] = 'OPERATIONAL'
    
    def publish_motor_diagnostics_custom(self):
        """Publish motor diagnostics using custom messages."""
        for motor_name, data in self.motor_data.items():
            msg = DriveMotorDiagnostic()
            msg.motor_name = motor_name
            msg.voltage = data['voltage']
            msg.current = data['current']
            msg.temperature = data['temperature']
            msg.status = data['status']
            msg.timestamp = self.get_clock().now().nanoseconds
            
            self.motor_diagnostics_pub.publish(msg)
    
    def publish_motor_diagnostics_standard(self):
        """Publish motor diagnostics using standard messages."""
        for motor_name, data in self.motor_data.items():
            msg = Float64MultiArray()
            # Format: [voltage, current, temperature, status_code]
            status_code = {'OK': 0, 'CAUTION': 1, 'WARNING': 2}.get(data['status'], 0)
            msg.data = [data['voltage'], data['current'], data['temperature'], float(status_code)]
            
            self.motor_diagnostics_pub.publish(msg)
    
    def publish_drive_status_custom(self):
        """Publish drive status using custom messages."""
        msg = DriveMotorStatus()
        msg.overall_status = self.drive_status['overall_status']
        msg.battery_level = self.drive_status['battery_level']
        msg.system_voltage = self.drive_status['system_voltage']
        msg.total_current = self.drive_status['total_current']
        msg.average_temperature = self.drive_status['average_temperature']
        msg.timestamp = self.get_clock().now().nanoseconds
        
        self.drive_status_pub.publish(msg)
    
    def publish_drive_status_standard(self):
        """Publish drive status using standard messages."""
        msg = Float64MultiArray()
        # Format: [battery_level, system_voltage, total_current, average_temperature, status_code]
        status_code = {'OPERATIONAL': 0, 'WARNING': 1, 'LOW_BATTERY': 2}.get(self.drive_status['overall_status'], 0)
        msg.data = [
            self.drive_status['battery_level'],
            self.drive_status['system_voltage'],
            self.drive_status['total_current'],
            self.drive_status['average_temperature'],
            float(status_code)
        ]
        
        self.drive_status_pub.publish(msg)
    
    def publish_cmd_vel(self):
        """Publish command velocity (simulate some movement)."""
        msg = Twist()
        
        # Simulate some movement patterns
        time_since_start = time.time()
        msg.linear.x = 0.5 * (1 + 0.3 * math.sin(time_since_start * 0.5))  # Varying forward speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1 * math.sin(time_since_start * 0.3)  # Gentle turning
        
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        publisher = MockDrivePublisher()
        print("Mock Drive Publisher started. Press Ctrl+C to stop.")
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\nMock Drive Publisher stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 