#!/usr/bin/env python3
"""
Mock ROS Drive Data Launcher

Launches the mock drive firmware node from the sim package to simulate
realistic drive motor data for testing. This provides a proper ROS-based
simulation that publishes to ROS topics which the frontend can consume
through the normal ROS pipeline.

Usage:
    python3 mock_ros_drive_data.py --scenario normal
    python3 mock_ros_drive_data.py --scenario motor_fault

Features:
- Uses the existing mock_drive_firmware_node.py from sim package
- Publishes proper ROS messages (DriveMotorDiagnostic, Float32MultiArray)
- Supports multiple test scenarios (normal, motor_fault, low_battery, overheating)
- Integrates with existing ROS infrastructure
- Configurable update rates and realistic physics simulation
"""

import subprocess
import sys
import os
import signal
import time
import logging
import argparse
from typing import Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class MockRosDriveDataLauncher:
    """Launcher for the mock ROS drive data simulation."""
    
    def __init__(self):
        self.ros_process: Optional[subprocess.Popen] = None
        self.running = False
    
    def check_ros_environment(self) -> bool:
        """Check if ROS2 environment is properly set up."""
        try:
            # Check if ROS2 is available
            result = subprocess.run(
                ["ros2", "--version"], 
                capture_output=True, 
                text=True, 
                timeout=5
            )
            if result.returncode != 0:
                logger.error("ROS2 not found or not working properly")
                return False
            
            logger.info(f"ROS2 version: {result.stdout.strip()}")
            
            # Check if custom message interfaces are available
            result = subprocess.run(
                ["ros2", "interface", "list"], 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            
            if "msg_srv_interface" not in result.stdout:
                logger.warning("Custom message interfaces (msg_srv_interface) not found")
                logger.warning("You may need to build the workspace: colcon build")
                # Don't fail here, the node might still work with basic messages
            
            return True
            
        except subprocess.TimeoutExpired:
            logger.error("ROS2 command timed out")
            return False
        except FileNotFoundError:
            logger.error("ROS2 not found. Please source your ROS2 environment.")
            return False
        except Exception as e:
            logger.error(f"Error checking ROS environment: {e}")
            return False
    
    def find_mock_node_path(self) -> Optional[str]:
        """Find the path to the mock drive firmware node."""
        # Get the current script directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Navigate to the sim package
        sim_path = os.path.join(os.path.dirname(current_dir), "sim", "mock_nodes", "mock_drive_firmware_node.py")
        
        if os.path.exists(sim_path):
            return sim_path
        
        # Alternative path if running from different location
        alt_sim_path = os.path.join(current_dir, "..", "..", "sim", "mock_nodes", "mock_drive_firmware_node.py")
        alt_sim_path = os.path.abspath(alt_sim_path)
        
        if os.path.exists(alt_sim_path):
            return alt_sim_path
        
        logger.error(f"Mock drive firmware node not found at {sim_path} or {alt_sim_path}")
        return None
    
    def launch_mock_node(self, scenario: str = "normal", 
                        diagnostics_rate: float = 10.0,
                        speeds_rate: float = 20.0,
                        enable_faults: bool = False,
                        realistic_physics: bool = True) -> bool:
        """Launch the mock drive firmware node."""
        
        node_path = self.find_mock_node_path()
        if not node_path:
            return False
        
        # Build the command to run the ROS node
        cmd = [
            "python3", node_path,
            "--ros-args",
            "-p", f"scenario:={scenario}",
            "-p", f"diagnostics_rate:={diagnostics_rate}",
            "-p", f"speeds_rate:={speeds_rate}",
            "-p", f"enable_faults:={enable_faults}",
            "-p", f"realistic_physics:={realistic_physics}"
        ]
        
        try:
            logger.info(f"Launching mock drive firmware node with scenario: {scenario}")
            logger.info(f"Command: {' '.join(cmd)}")
            
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Give the node a moment to start
            time.sleep(2)
            
            # Check if the process is still running
            if self.ros_process.poll() is None:
                logger.info(f"Mock drive firmware node started successfully (PID: {self.ros_process.pid})")
                self.running = True
                return True
            else:
                # Process exited, get the output
                stdout, _ = self.ros_process.communicate()
                logger.error(f"Mock drive firmware node failed to start. Output:\n{stdout}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to launch mock drive firmware node: {e}")
            return False
    
    def monitor_node(self):
        """Monitor the running node and log its output."""
        if not self.ros_process:
            return
        
        logger.info("Monitoring mock drive firmware node output...")
        logger.info("Press Ctrl+C to stop the simulation")
        
        try:
            # Read output line by line
            for line in iter(self.ros_process.stdout.readline, ''):
                if line:
                    # Filter and format ROS node output
                    line = line.strip()
                    if line:
                        # Skip some verbose ROS messages
                        if not any(skip in line for skip in ['[INFO]', '[DEBUG]', 'rcl_logging']):
                            logger.info(f"Node: {line}")
                
                # Check if process is still running
                if self.ros_process.poll() is not None:
                    break
                    
        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error(f"Error monitoring node: {e}")
    
    def stop_node(self):
        """Stop the running mock node."""
        if self.ros_process and self.running:
            logger.info("Stopping mock drive firmware node...")
            
            try:
                # Send SIGTERM first
                self.ros_process.terminate()
                
                # Wait for graceful shutdown
                try:
                    self.ros_process.wait(timeout=5)
                    logger.info("Mock drive firmware node stopped gracefully")
                except subprocess.TimeoutExpired:
                    # Force kill if needed
                    logger.warning("Node didn't stop gracefully, forcing termination")
                    self.ros_process.kill()
                    self.ros_process.wait()
                    logger.info("Mock drive firmware node terminated")
                    
            except Exception as e:
                logger.error(f"Error stopping node: {e}")
            finally:
                self.running = False
                self.ros_process = None
    
    def run(self, scenario: str = "normal", 
            diagnostics_rate: float = 10.0,
            speeds_rate: float = 20.0,
            enable_faults: bool = False,
            realistic_physics: bool = True):
        """Run the mock ROS drive data simulation."""
        
        # Check ROS environment
        if not self.check_ros_environment():
            logger.error("ROS environment check failed")
            return False
        
        # Launch the mock node
        if not self.launch_mock_node(scenario, diagnostics_rate, speeds_rate, enable_faults, realistic_physics):
            logger.error("Failed to launch mock node")
            return False
        
        try:
            # Monitor the node
            self.monitor_node()
            
        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
        except Exception as e:
            logger.error(f"Simulation error: {e}")
        finally:
            self.stop_node()
        
        return True


def signal_handler(signum, frame):
    """Handle interrupt signals."""
    logger.info("Received interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Mock ROS Drive Data Launcher")
    parser.add_argument(
        "--scenario",
        default="normal",
        choices=["normal", "motor_fault", "low_battery", "overheating"],
        help="Test scenario to simulate"
    )
    parser.add_argument(
        "--diagnostics-rate",
        type=float,
        default=10.0,
        help="Diagnostics publishing rate (Hz)"
    )
    parser.add_argument(
        "--speeds-rate",
        type=float,
        default=20.0,
        help="Speeds publishing rate (Hz)"
    )
    parser.add_argument(
        "--enable-faults",
        action="store_true",
        help="Enable random fault injection"
    )
    parser.add_argument(
        "--disable-physics",
        action="store_true",
        help="Disable realistic physics simulation"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("🚀 Starting Mock ROS Drive Data Simulation")
    logger.info(f"Scenario: {args.scenario}")
    logger.info(f"Diagnostics Rate: {args.diagnostics_rate} Hz")
    logger.info(f"Speeds Rate: {args.speeds_rate} Hz")
    logger.info(f"Enable Faults: {args.enable_faults}")
    logger.info(f"Realistic Physics: {not args.disable_physics}")
    
    # Create and run launcher
    launcher = MockRosDriveDataLauncher()
    success = launcher.run(
        scenario=args.scenario,
        diagnostics_rate=args.diagnostics_rate,
        speeds_rate=args.speeds_rate,
        enable_faults=args.enable_faults,
        realistic_physics=not args.disable_physics
    )
    
    if success:
        logger.info("✅ Mock ROS drive data simulation completed")
        sys.exit(0)
    else:
        logger.error("❌ Mock ROS drive data simulation failed")
        sys.exit(1)


if __name__ == "__main__":
    main()
