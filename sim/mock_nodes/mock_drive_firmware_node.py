#!/usr/bin/env python3
"""
Mock Drive Firmware Node

Simulates the drive firmware node for testing without hardware.
Publishes realistic motor diagnostic data and provides service interfaces
that match the real firmware node behavior.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray
import random
import math
import time
import yaml
import os
from typing import Dict, List, Optional

# Import custom message types
try:
    from msg_srv_interface.msg import DriveMotorDiagnostic
    from msg_srv_interface.srv import DriveMotorStatus
except ImportError:
    print("Warning: Custom message interfaces not found. Make sure msg_srv_interface is built.")
    DriveMotorDiagnostic = None
    DriveMotorStatus = None


class MockDriveFirmwareNode(Node):
    """
    Mock implementation of the drive firmware node.
    
    Simulates realistic motor behavior including:
    - Motor diagnostics (voltage, current, temperature)
    - Motor speeds
    - Connection status
    - Various failure modes for testing
    """
    
    def __init__(self):
        super().__init__('mock_drive_firmware_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('config_file', ''),
                ('scenario', 'normal'),
                ('diagnostics_rate', 10.0),
                ('speeds_rate', 20.0),
                ('enable_faults', False),
                ('fault_probability', 0.01),
                ('realistic_physics', True),
            ]
        )
        
        # Load configuration
        self.config = self._load_config()
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        
        # Motor names (RF, RB, LB, LF)
        self.motor_names = ['RF', 'RB', 'LB', 'LF']
        
        # Initialize motor states
        self.motor_states = {
            motor: {
                'voltage': 12.0,
                'current': 0.0,
                'temperature': 25.0,
                'speed': 0.0,
                'connected': True,
                'target_speed': 0.0,
                'fault_active': False,
                'fault_type': None,
            }
            for motor in self.motor_names
        }
        
        # Simulation state
        self.simulation_time = 0.0
        self.last_update = time.time()
        
        # Publishers
        if DriveMotorDiagnostic:
            self.diagnostics_pub = self.create_publisher(
                DriveMotorDiagnostic,
                'drive_motors_info',
                10
            )
        
        self.speeds_pub = self.create_publisher(
            Float32MultiArray,
            'drive_speeds_info',
            10
        )
        
        # Service server
        if DriveMotorStatus:
            self.status_service = self.create_service(
                DriveMotorStatus,
                'drive_motors_status',
                self.handle_motor_status_request
            )
        
        # Timers
        diagnostics_rate = self.get_parameter('diagnostics_rate').get_parameter_value().double_value
        speeds_rate = self.get_parameter('speeds_rate').get_parameter_value().double_value
        
        self.diagnostics_timer = self.create_timer(
            1.0 / diagnostics_rate,
            self.publish_diagnostics
        )
        
        self.speeds_timer = self.create_timer(
            1.0 / speeds_rate,
            self.publish_speeds
        )
        
        # Update timer for physics simulation
        self.update_timer = self.create_timer(0.05, self.update_simulation)
        
        self.get_logger().info(f'Mock Drive Firmware Node started with scenario: {self.scenario}')
        self.get_logger().info(f'Publishing diagnostics at {diagnostics_rate} Hz')
        self.get_logger().info(f'Publishing speeds at {speeds_rate} Hz')
    
    def _load_config(self) -> Dict:
        """Load configuration from YAML file or use defaults."""
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        default_config = {
            'scenarios': {
                'normal': {
                    'voltage_range': [11.5, 12.5],
                    'current_max': 3.0,
                    'temp_range': [20.0, 45.0],
                    'speed_max': 100.0,
                    'all_motors_connected': True,
                },
                'low_battery': {
                    'voltage_range': [9.0, 10.5],
                    'current_max': 2.0,
                    'temp_range': [20.0, 40.0],
                    'speed_max': 60.0,
                    'all_motors_connected': True,
                },
                'motor_fault': {
                    'voltage_range': [11.5, 12.5],
                    'current_max': 3.0,
                    'temp_range': [20.0, 80.0],
                    'speed_max': 100.0,
                    'all_motors_connected': False,
                    'faulty_motors': ['RB'],
                },
                'overheating': {
                    'voltage_range': [11.5, 12.5],
                    'current_max': 4.0,
                    'temp_range': [60.0, 85.0],
                    'speed_max': 100.0,
                    'all_motors_connected': True,
                },
            }
        }
        
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    loaded_config = yaml.safe_load(f)
                    default_config.update(loaded_config)
            except Exception as e:
                self.get_logger().warn(f'Failed to load config file {config_file}: {e}')
        
        return default_config
    
    def update_simulation(self):
        """Update the physics simulation of motor behavior."""
        try:
            current_time = time.time()
            dt = current_time - self.last_update
            self.last_update = current_time
            self.simulation_time += dt
            
            scenario_config = self.config['scenarios'].get(self.scenario, self.config['scenarios']['normal'])
            realistic_physics = self.get_parameter('realistic_physics').get_parameter_value().bool_value
            
            for motor_name in self.motor_names:
                motor = self.motor_states[motor_name]
                
                # Check if motor should be disconnected for this scenario
                if not scenario_config.get('all_motors_connected', True):
                    faulty_motors = scenario_config.get('faulty_motors', [])
                    if motor_name in faulty_motors:
                        motor['connected'] = False
                        motor['voltage'] = 0.0
                        motor['current'] = 0.0
                        motor['speed'] = 0.0
                        continue
                
                # Simulate realistic motor physics
                if realistic_physics:
                    self._simulate_motor_physics(motor, scenario_config, dt)
                else:
                    self._simulate_basic_motor(motor, scenario_config)
                
                # Inject faults if enabled
                if self.get_parameter('enable_faults').get_parameter_value().bool_value:
                    self._inject_random_faults(motor)

        except Exception as e:
            self.get_logger().error(f"Simulation update failed: {e}")
    
    def _simulate_motor_physics(self, motor: Dict, config: Dict, dt: float):
        """Simulate realistic motor physics."""
        # Add some dynamic target speed changes
        base_speed = 30.0 * math.sin(self.simulation_time * 0.1)  # Slow oscillation
        noise = random.uniform(-5.0, 5.0)  # Random noise
        motor['target_speed'] = max(0, base_speed + noise)
        
        # Simulate motor acceleration/deceleration
        speed_diff = motor['target_speed'] - motor['speed']
        acceleration = 20.0  # rad/s^2
        max_change = acceleration * dt
        
        if abs(speed_diff) > max_change:
            motor['speed'] += max_change if speed_diff > 0 else -max_change
        else:
            motor['speed'] = motor['target_speed']
        
        # Current is proportional to load and acceleration
        base_current = abs(motor['speed']) * 0.02  # Base current for speed
        accel_current = abs(speed_diff) * 0.1  # Additional current for acceleration
        motor['current'] = min(base_current + accel_current, config['current_max'])
        
        # Voltage drops under load
        voltage_drop = motor['current'] * 0.2
        base_voltage = random.uniform(*config['voltage_range'])
        motor['voltage'] = max(9.0, base_voltage - voltage_drop)
        
        # Temperature increases with current and time
        ambient_temp = config['temp_range'][0]
        heat_from_current = motor['current'] * 8.0
        heat_from_time = min(self.simulation_time * 0.5, 20.0)  # Gradual warmup
        motor['temperature'] = ambient_temp + heat_from_current + heat_from_time
        
        # Clamp temperature to scenario range
        motor['temperature'] = min(motor['temperature'], config['temp_range'][1])
    
    def _simulate_basic_motor(self, motor: Dict, config: Dict):
        """Simple motor simulation without physics."""
        motor['voltage'] = random.uniform(*config['voltage_range'])
        motor['current'] = random.uniform(0, config['current_max'])
        motor['temperature'] = random.uniform(*config['temp_range'])
        motor['speed'] = random.uniform(0, config['speed_max'])
    
    def _inject_random_faults(self, motor: Dict):
        """Randomly inject faults for testing error handling."""
        fault_prob = self.get_parameter('fault_probability').get_parameter_value().double_value
        
        if random.random() < fault_prob:
            fault_types = ['overcurrent', 'overheat', 'disconnect']
            fault_type = random.choice(fault_types)
            
            if fault_type == 'overcurrent':
                motor['current'] = 5.0
                motor['fault_active'] = True
                motor['fault_type'] = 'overcurrent'
            elif fault_type == 'overheat':
                motor['temperature'] = 90.0
                motor['fault_active'] = True
                motor['fault_type'] = 'overheat'
            elif fault_type == 'disconnect':
                motor['connected'] = False
                motor['fault_active'] = True
                motor['fault_type'] = 'disconnect'
        
        # Clear faults randomly
        elif motor['fault_active'] and random.random() < 0.1:
            motor['fault_active'] = False
            motor['fault_type'] = None
            motor['connected'] = True
    
    def publish_diagnostics(self):
        """Publish motor diagnostic information."""
        if not DriveMotorDiagnostic:
            return
        
        try:
            msg = DriveMotorDiagnostic()
            
            for motor_name in self.motor_names:
                motor = self.motor_states[motor_name]
                
                # Motor state: 1.0 = connected, 0.0 = disconnected
                state = 1.0 if motor['connected'] else 0.0
                
                if motor_name == 'RF':
                    msg.rf_voltage = float(motor['voltage'])
                    msg.rf_current = float(motor['current'])
                    msg.rf_temperature = float(motor['temperature'])
                    msg.rf_state = state
                elif motor_name == 'RB':
                    msg.rb_voltage = float(motor['voltage'])
                    msg.rb_current = float(motor['current'])
                    msg.rb_temperature = float(motor['temperature'])
                    msg.rb_state = state
                elif motor_name == 'LB':
                    msg.lb_voltage = float(motor['voltage'])
                    msg.lb_current = float(motor['current'])
                    msg.lb_temperature = float(motor['temperature'])
                    msg.lb_state = state
                elif motor_name == 'LF':
                    msg.lf_voltage = float(motor['voltage'])
                    msg.lf_current = float(motor['current'])
                    msg.lf_temperature = float(motor['temperature'])
                    msg.lf_state = state
            
            self.diagnostics_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish diagnostics: {e}")

    def publish_speeds(self):
        """Publish motor speed information."""
        msg = Float32MultiArray()

        try:
            speeds = [
                float(self.motor_states[motor].get('speed', 0.0))
                for motor in self.motor_names
            ]
            msg.data = speeds
            self.speeds_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish speeds: {e}")

    
    def handle_motor_status_request(self, request, response):
        """Handle motor status service requests."""
        if not DriveMotorStatus:
            return response
        
        # Return connection status for all motors
        response.rf_ok = self.motor_states['RF']['connected']
        response.rb_ok = self.motor_states['RB']['connected']
        response.lb_ok = self.motor_states['LB']['connected']
        response.lf_ok = self.motor_states['LF']['connected']
        
        return response


def main(args=None):
    """Main entry point for the mock drive firmware node."""
    rclpy.init(args=args)
    
    try:
        node = MockDriveFirmwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in mock drive firmware node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
