#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
import math
import json
import time
from typing import Optional, Dict, Any

class GPSService(Node):
    """
    GPS Service for the rover mapping interface.
    Integrates with ROS GPS data and provides a REST API for the web interface.
    """
    
    def __init__(self):
        super().__init__('gps_service')
        
        # GPS data storage
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'heading': 0.0,
            'accuracy': 0.0,
            'timestamp': 0,
            'fix_quality': 0,
            'satellites': 0
        }
        
        # ROS subscribers
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Adjust topic name as needed
            self.gps_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Adjust topic name as needed
            self.imu_callback,
            10
        )
        
        # Optional: velocity for heading calculation
        self.velocity_subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',  # Adjust topic name as needed
            self.velocity_callback,
            10
        )
        
        # Timer for periodic GPS data updates
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('GPS Service started')
        
        # Store last velocity for heading calculation
        self.last_velocity = {'linear_x': 0.0, 'linear_y': 0.0}
        self.last_imu_yaw = 0.0
        
    def gps_callback(self, msg: NavSatFix):
        """Handle GPS fix messages"""
        self.gps_data['latitude'] = msg.latitude
        self.gps_data['longitude'] = msg.longitude
        self.gps_data['accuracy'] = self.calculate_accuracy(msg)
        self.gps_data['timestamp'] = int(time.time() * 1000)  # Convert to milliseconds
        self.gps_data['fix_quality'] = msg.status.status
        self.gps_data['satellites'] = len(msg.status.service) if hasattr(msg.status, 'service') else 0
        
        self.get_logger().debug(f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f}')
        
    def imu_callback(self, msg: Imu):
        """Handle IMU messages for heading calculation"""
        # Extract yaw from quaternion
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert quaternion to euler angles (yaw)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_degrees = math.degrees(yaw)
        
        # Normalize to 0-360 degrees
        yaw_degrees = (yaw_degrees + 360) % 360
        
        self.last_imu_yaw = yaw_degrees
        self.gps_data['heading'] = yaw_degrees
        
    def velocity_callback(self, msg: TwistStamped):
        """Handle velocity messages for heading calculation"""
        self.last_velocity['linear_x'] = msg.twist.linear.x
        self.last_velocity['linear_y'] = msg.twist.linear.y
        
        # Calculate heading from velocity if moving
        speed = math.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
        if speed > 0.1:  # Only calculate heading if moving
            heading = math.degrees(math.atan2(msg.twist.linear.y, msg.twist.linear.x))
            heading = (heading + 360) % 360
            self.gps_data['heading'] = heading
    
    def calculate_accuracy(self, msg: NavSatFix) -> float:
        """Calculate GPS accuracy based on position covariance"""
        if msg.position_covariance_type == 0:  # COVARIANCE_TYPE_UNKNOWN
            return 50.0  # Default accuracy in meters
        
        # Extract diagonal elements of covariance matrix
        var_x = msg.position_covariance[0]
        var_y = msg.position_covariance[4]
        
        # Calculate 95% confidence interval (2-sigma)
        accuracy = 2.0 * math.sqrt(max(var_x, var_y))
        
        return max(accuracy, 1.0)  # Minimum accuracy of 1 meter
    
    def timer_callback(self):
        """Periodic callback to update GPS data"""
        # Update timestamp
        self.gps_data['timestamp'] = int(time.time() * 1000)
        
        # Log GPS status periodically
        if self.gps_data['fix_quality'] > 0:
            self.get_logger().info(
                f'GPS: {self.gps_data["latitude"]:.6f}, {self.gps_data["longitude"]:.6f}, '
                f'Heading: {self.gps_data["heading"]:.1f}°, Accuracy: {self.gps_data["accuracy"]:.1f}m'
            )
    
    def get_gps_data(self) -> Dict[str, Any]:
        """Get current GPS data"""
        return self.gps_data.copy()
    
    def get_gps_status(self) -> Dict[str, Any]:
        """Get GPS status information"""
        status = {
            'has_fix': self.gps_data['fix_quality'] > 0,
            'fix_quality': self.gps_data['fix_quality'],
            'satellites': self.gps_data['satellites'],
            'accuracy': self.gps_data['accuracy'],
            'last_update': self.gps_data['timestamp']
        }
        return status

def main(args=None):
    rclpy.init(args=args)
    
    gps_service = GPSService()
    
    try:
        rclpy.spin(gps_service)
    except KeyboardInterrupt:
        pass
    finally:
        gps_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 