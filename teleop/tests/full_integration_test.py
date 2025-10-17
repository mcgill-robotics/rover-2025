#!/usr/bin/env python3
"""
Full Integration Test for Teleop System

Comprehensive test that combines camera system testing with ROS drive data testing.
Tests the complete /drive page functionality including both camera feeds and
motor diagnostics/information panels.

Usage:
    python3 full_integration_test.py

Features:
- Tests complete camera management system (mock Jetson devices)
- Tests ROS drive data pipeline (motor diagnostics, speeds, status)
- Tests frontend integration with both systems
- Validates data accuracy and system behavior
- Supports multiple test scenarios
"""

import unittest
import asyncio
import aiohttp
import websockets
import json
import time
import subprocess
import signal
import os
import sys
import threading
import queue
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
import tempfile

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class TestConfig:
    """Configuration for integration tests."""
    central_backend_url: str = "http://localhost:8081"
    central_backend_ws: str = "ws://localhost:8081/jetson"
    ros_manager_url: str = "http://localhost:8082"
    ros_manager_ws: str = "ws://localhost:8082/ws"
    frontend_url: str = "http://localhost:3000"
    test_timeout: int = 60
    startup_delay: int = 5

class ServiceManager:
    """Manages test services (mock servers, backends, etc.)."""
    
    def __init__(self, config: TestConfig):
        self.config = config
        self.processes = {}
        self.temp_files = []
    
    def start_service(self, name: str, command: List[str], cwd: str = None) -> bool:
        """Start a service process."""
        try:
            logger.info(f"Starting {name}...")
            
            # Create log files
            log_file = tempfile.NamedTemporaryFile(
                mode='w+', 
                prefix=f'{name}_', 
                suffix='.log', 
                delete=False
            )
            self.temp_files.append(log_file.name)
            
            process = subprocess.Popen(
                command,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                cwd=cwd,
                preexec_fn=os.setsid
            )
            
            self.processes[name] = {
                'process': process,
                'log_file': log_file.name
            }
            
            logger.info(f"{name} started with PID {process.pid}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start {name}: {e}")
            return False
    
    def stop_service(self, name: str):
        """Stop a service process."""
        if name in self.processes:
            try:
                process_info = self.processes[name]
                process = process_info['process']
                
                logger.info(f"Stopping {name} (PID {process.pid})...")
                
                # Send SIGTERM to process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                
                # Wait for graceful shutdown
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if needed
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()
                
                logger.info(f"{name} stopped")
                del self.processes[name]
                
            except Exception as e:
                logger.error(f"Error stopping {name}: {e}")
    
    def stop_all_services(self):
        """Stop all running services."""
        for name in list(self.processes.keys()):
            self.stop_service(name)
    
    def get_service_logs(self, name: str) -> str:
        """Get logs for a service."""
        if name in self.processes:
            log_file = self.processes[name]['log_file']
            try:
                with open(log_file, 'r') as f:
                    return f.read()
            except Exception as e:
                return f"Error reading logs: {e}"
        return "Service not found"
    
    def cleanup(self):
        """Clean up resources."""
        self.stop_all_services()
        
        # Clean up temp files
        for temp_file in self.temp_files:
            try:
                os.unlink(temp_file)
            except Exception:
                pass

class FullIntegrationTest(unittest.TestCase):
    """Full integration test for the teleop system."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        cls.config = TestConfig()
        cls.service_manager = ServiceManager(cls.config)
        cls.test_data = {}
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment."""
        cls.service_manager.cleanup()
    
    def setUp(self):
        """Set up individual test."""
        self.received_data = queue.Queue()
        self.websocket_connections = {}
        
    def tearDown(self):
        """Clean up individual test."""
        # Close any open WebSocket connections
        for ws in self.websocket_connections.values():
            if ws and not ws.closed:
                asyncio.create_task(ws.close())
    
    def start_test_services(self) -> bool:
        """Start all required test services."""
        services_started = []
        
        try:
            # Get the test directory path
            test_dir = os.path.dirname(os.path.abspath(__file__))
            
            # Start mock Jetson server
            if self.service_manager.start_service(
                "mock_jetson_server",
                ["python3", "mock_jetson_server.py", "--backend-url", self.config.central_backend_ws],
                cwd=test_dir
            ):
                services_started.append("mock_jetson_server")
            
            # Start mock ROS drive data simulator (now uses ROS node)
            if self.service_manager.start_service(
                "mock_ros_drive_data",
                ["python3", "mock_ros_drive_data.py", "--scenario", "normal"],
                cwd=test_dir
            ):
                services_started.append("mock_ros_drive_data")
            
            # Start central backend (camera system)
            services_dir = os.path.join(os.path.dirname(test_dir), "services", "camera")
            if self.service_manager.start_service(
                "central_backend",
                ["python3", "central_backend.py"],
                cwd=services_dir
            ):
                services_started.append("central_backend")
            
            # Start ROS manager (drive system)
            ros_services_dir = os.path.join(os.path.dirname(test_dir), "services", "ros")
            if os.path.exists(os.path.join(ros_services_dir, "ros_manager.py")):
                if self.service_manager.start_service(
                    "ros_manager",
                    ["python3", "ros_manager.py"],
                    cwd=ros_services_dir
                ):
                    services_started.append("ros_manager")
            
            # Wait for services to start
            logger.info(f"Waiting {self.config.startup_delay} seconds for services to start...")
            time.sleep(self.config.startup_delay)
            
            # Verify services are responding
            return self.verify_services_health()
            
        except Exception as e:
            logger.error(f"Error starting test services: {e}")
            return False
    
    def verify_services_health(self) -> bool:
        """Verify that all services are healthy and responding."""
        health_checks = []
        
        # Check central backend
        try:
            response = subprocess.run(
                ["curl", "-s", f"{self.config.central_backend_url}/health"],
                capture_output=True,
                timeout=5
            )
            if response.returncode == 0:
                health_checks.append("central_backend")
                logger.info("✅ Central backend is healthy")
            else:
                logger.warning("❌ Central backend health check failed")
        except Exception as e:
            logger.warning(f"Central backend health check error: {e}")
        
        # Check ROS manager
        try:
            response = subprocess.run(
                ["curl", "-s", f"{self.config.ros_manager_url}/api/health"],
                capture_output=True,
                timeout=5
            )
            if response.returncode == 0:
                health_checks.append("ros_manager")
                logger.info("✅ ROS manager is healthy")
            else:
                logger.warning("❌ ROS manager health check failed")
        except Exception as e:
            logger.warning(f"ROS manager health check error: {e}")
        
        return len(health_checks) >= 1  # At least one service should be healthy
    
    async def test_camera_system_integration(self):
        """Test camera system integration."""
        logger.info("Testing camera system integration...")
        
        # Connect to central backend WebSocket
        try:
            ws_url = f"ws://localhost:8081/ws"
            websocket = await websockets.connect(ws_url)
            self.websocket_connections['camera'] = websocket
            
            # Wait for device discovery messages
            discovery_messages = []
            start_time = time.time()
            
            while time.time() - start_time < 10:  # 10 second timeout
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(message)
                    
                    if data.get("type") == "device_discovery":
                        discovery_messages.append(data)
                        logger.info(f"Received device discovery: {data['device_id']}")
                        
                        # Stop after receiving expected devices
                        if len(discovery_messages) >= 3:  # Expecting 3 Jetson devices
                            break
                            
                except asyncio.TimeoutError:
                    continue
                except json.JSONDecodeError:
                    continue
            
            # Verify we received device discoveries
            self.assertGreaterEqual(len(discovery_messages), 1, "Should receive at least one device discovery")
            
            # Test camera start command
            if discovery_messages:
                device = discovery_messages[0]
                device_id = device["device_id"]
                cameras = device["device_info"]["cameras"]
                
                if cameras:
                    camera_id = cameras[0]["camera_id"]
                    
                    # Send start command
                    start_command = {
                        "type": "camera_command",
                        "device_id": device_id,
                        "camera_id": camera_id,
                        "command": "start",
                        "timestamp": time.time()
                    }
                    
                    await websocket.send(json.dumps(start_command))
                    logger.info(f"Sent start command for {device_id}/{camera_id}")
                    
                    # Wait for acknowledgment and frame data
                    ack_received = False
                    frames_received = 0
                    start_time = time.time()
                    
                    while time.time() - start_time < 10:
                        try:
                            message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                            data = json.loads(message)
                            
                            if data.get("type") == "camera_command_ack":
                                ack_received = True
                                logger.info("Received camera command acknowledgment")
                            elif data.get("type") == "camera_frame":
                                frames_received += 1
                                if frames_received >= 3:  # Received a few frames
                                    break
                                    
                        except asyncio.TimeoutError:
                            continue
                        except json.JSONDecodeError:
                            continue
                    
                    self.assertTrue(ack_received, "Should receive camera command acknowledgment")
                    self.assertGreater(frames_received, 0, "Should receive camera frames")
                    
                    logger.info(f"✅ Camera system test passed - received {frames_received} frames")
            
        except Exception as e:
            self.fail(f"Camera system integration test failed: {e}")
    
    async def test_ros_drive_system_integration(self):
        """Test ROS drive system integration."""
        logger.info("Testing ROS drive system integration...")
        
        # Test REST API endpoints
        endpoints = [
            "/api/drive/summary",
            "/api/drive/diagnostics", 
            "/api/drive/speeds",
            "/api/drive/status"
        ]
        
        async with aiohttp.ClientSession() as session:
            for endpoint in endpoints:
                try:
                    url = f"{self.config.ros_manager_url}{endpoint}"
                    async with session.get(url) as response:
                        self.assertEqual(response.status, 200, f"Endpoint {endpoint} should return 200")
                        
                        data = await response.json()
                        self.assertIsInstance(data, dict, f"Endpoint {endpoint} should return JSON object")
                        
                        logger.info(f"✅ Endpoint {endpoint} responded correctly")
                        
                except Exception as e:
                    self.fail(f"REST API test failed for {endpoint}: {e}")
        
        # Test WebSocket connection
        try:
            websocket = await websockets.connect(self.config.ros_manager_ws)
            self.websocket_connections['ros'] = websocket
            
            # Subscribe to drive topics
            subscribe_msg = {
                "type": "subscribe",
                "topics": ["drive_diagnostics", "drive_speeds", "drive_status"],
                "timestamp": time.time()
            }
            
            await websocket.send(json.dumps(subscribe_msg))
            logger.info("Sent subscription request")
            
            # Collect data for a few seconds
            data_received = {
                "drive_diagnostics": 0,
                "drive_speeds": 0,
                "drive_status": 0
            }
            
            start_time = time.time()
            while time.time() - start_time < 10:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(message)
                    
                    if data.get("type") == "data_update":
                        topic = data.get("topic")
                        if topic in data_received:
                            data_received[topic] += 1
                            
                            # Validate data structure
                            if topic == "drive_diagnostics":
                                motors = data.get("data", {}).get("motors", {})
                                self.assertIn("RF", motors, "Should have RF motor data")
                                self.assertIn("voltage", motors["RF"], "Should have voltage data")
                                
                            elif topic == "drive_speeds":
                                speeds = data.get("data", {}).get("speeds", {})
                                self.assertIn("RF", speeds, "Should have RF speed data")
                                
                            elif topic == "drive_status":
                                status = data.get("data", {}).get("status", {})
                                self.assertIn("RF", status, "Should have RF status data")
                    
                    # Stop after receiving some data from each topic
                    if all(count > 0 for count in data_received.values()):
                        break
                        
                except asyncio.TimeoutError:
                    continue
                except json.JSONDecodeError:
                    continue
            
            # Verify we received data from all topics
            for topic, count in data_received.items():
                self.assertGreater(count, 0, f"Should receive data from {topic}")
                logger.info(f"✅ Received {count} messages from {topic}")
            
        except Exception as e:
            self.fail(f"ROS drive system integration test failed: {e}")
    
    async def test_combined_system_integration(self):
        """Test both camera and drive systems working together."""
        logger.info("Testing combined system integration...")
        
        # This test verifies that both systems can run simultaneously
        # without interfering with each other
        
        tasks = [
            asyncio.create_task(self.test_camera_system_integration()),
            asyncio.create_task(self.test_ros_drive_system_integration())
        ]
        
        try:
            # Run both tests concurrently
            await asyncio.gather(*tasks)
            logger.info("✅ Combined system integration test passed")
            
        except Exception as e:
            self.fail(f"Combined system integration test failed: {e}")
    
    def test_full_integration_normal_scenario(self):
        """Test full integration with normal scenario."""
        logger.info("🧪 Running full integration test - normal scenario")
        
        # Start all test services
        self.assertTrue(self.start_test_services(), "Failed to start test services")
        
        # Run async tests
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.test_combined_system_integration())
            logger.info("✅ Full integration test passed")
            
        except Exception as e:
            # Print service logs for debugging
            logger.error("❌ Full integration test failed")
            for service_name in self.service_manager.processes.keys():
                logs = self.service_manager.get_service_logs(service_name)
                logger.error(f"\n=== {service_name} logs ===\n{logs}\n")
            raise e
            
        finally:
            loop.close()
    
    def test_camera_only_integration(self):
        """Test camera system integration only."""
        logger.info("🧪 Running camera-only integration test")
        
        # Start only camera-related services
        test_dir = os.path.dirname(os.path.abspath(__file__))
        services_dir = os.path.join(os.path.dirname(test_dir), "services", "camera")
        
        services_started = []
        
        # Start mock Jetson server
        if self.service_manager.start_service(
            "mock_jetson_server",
            ["python3", "mock_jetson_server.py", "--backend-url", self.config.central_backend_ws],
            cwd=test_dir
        ):
            services_started.append("mock_jetson_server")
        
        # Start central backend
        if self.service_manager.start_service(
            "central_backend",
            ["python3", "central_backend.py"],
            cwd=services_dir
        ):
            services_started.append("central_backend")
        
        self.assertGreater(len(services_started), 0, "Should start at least one service")
        
        # Wait for startup
        time.sleep(self.config.startup_delay)
        
        # Run camera tests
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.test_camera_system_integration())
            logger.info("✅ Camera-only integration test passed")
            
        finally:
            loop.close()
    
    def test_drive_only_integration(self):
        """Test drive system integration only."""
        logger.info("🧪 Running drive-only integration test")
        
        # Start only drive-related services
        test_dir = os.path.dirname(os.path.abspath(__file__))
        ros_services_dir = os.path.join(os.path.dirname(test_dir), "services", "ros")
        
        services_started = []
        
        # Start mock ROS drive data simulator
        if self.service_manager.start_service(
            "mock_ros_drive_data",
            ["python3", "mock_ros_drive_data.py", "--ros-manager-url", self.config.ros_manager_ws],
            cwd=test_dir
        ):
            services_started.append("mock_ros_drive_data")
        
        # Start ROS manager if it exists
        if os.path.exists(os.path.join(ros_services_dir, "ros_manager.py")):
            if self.service_manager.start_service(
                "ros_manager",
                ["python3", "ros_manager.py"],
                cwd=ros_services_dir
            ):
                services_started.append("ros_manager")
        
        self.assertGreater(len(services_started), 0, "Should start at least one service")
        
        # Wait for startup
        time.sleep(self.config.startup_delay)
        
        # Run drive tests
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.test_ros_drive_system_integration())
            logger.info("✅ Drive-only integration test passed")
            
        finally:
            loop.close()

class TestRunner:
    """Custom test runner for integration tests."""
    
    def __init__(self):
        self.results = []
    
    def run_tests(self, test_types: List[str] = None):
        """Run integration tests."""
        if test_types is None:
            test_types = ["full", "camera", "drive"]
        
        suite = unittest.TestSuite()
        
        # Add tests based on requested types
        if "full" in test_types:
            suite.addTest(FullIntegrationTest('test_full_integration_normal_scenario'))
        
        if "camera" in test_types:
            suite.addTest(FullIntegrationTest('test_camera_only_integration'))
        
        if "drive" in test_types:
            suite.addTest(FullIntegrationTest('test_drive_only_integration'))
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        return result.wasSuccessful()

def signal_handler(signum, frame):
    """Handle interrupt signals."""
    logger.info("Received interrupt signal, shutting down...")
    sys.exit(0)

def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Full Integration Test for Teleop System")
    parser.add_argument(
        "--test-types",
        nargs="+",
        default=["full"],
        choices=["full", "camera", "drive"],
        help="Types of tests to run"
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
    
    logger.info("🚀 Starting Full Integration Tests")
    logger.info(f"Test types: {args.test_types}")
    
    # Run tests
    test_runner = TestRunner()
    success = test_runner.run_tests(args.test_types)
    
    if success:
        logger.info("🎉 All integration tests passed!")
        sys.exit(0)
    else:
        logger.error("❌ Some integration tests failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
