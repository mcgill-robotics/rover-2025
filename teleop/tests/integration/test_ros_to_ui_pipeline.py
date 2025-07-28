#!/usr/bin/env python3
"""
Integration test for the complete ROS to UI data pipeline.

Tests the full data flow from mock firmware nodes through ROS Manager
to the web UI, validating data accuracy and system behavior.
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
from typing import Dict, List, Optional, Any
import threading
import queue


class ROSToUIPipelineTest(unittest.TestCase):
    """Test the complete data pipeline from ROS to UI."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        cls.ros_manager_process = None
        cls.mock_firmware_process = None
        cls.test_timeout = 30  # seconds
        cls.ros_manager_url = "http://localhost:8082"
        cls.websocket_url = "ws://localhost:8082/ws"
        
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment."""
        cls._stop_processes()
    
    def setUp(self):
        """Set up individual test."""
        self.received_data = queue.Queue()
        self.websocket_connected = False
        
    def tearDown(self):
        """Clean up individual test."""
        pass
    
    @classmethod
    def _stop_processes(cls):
        """Stop all test processes."""
        if cls.ros_manager_process:
            cls.ros_manager_process.terminate()
            cls.ros_manager_process.wait(timeout=5)
            cls.ros_manager_process = None
            
        if cls.mock_firmware_process:
            cls.mock_firmware_process.terminate()
            cls.mock_firmware_process.wait(timeout=5)
            cls.mock_firmware_process = None
    
    def _start_ros_manager(self):
        """Start the ROS Manager service."""
        ros_manager_path = os.path.join(
            os.path.dirname(__file__), 
            "..", "..", "services", "ros", "ros_manager.py"
        )
        
        if not os.path.exists(ros_manager_path):
            self.skipTest(f"ROS Manager not found at {ros_manager_path}")
        
        self.ros_manager_process = subprocess.Popen(
            ["python3", ros_manager_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        # Wait for ROS Manager to start
        for _ in range(30):  # 30 second timeout
            try:
                response = subprocess.run(
                    ["curl", "-s", f"{self.ros_manager_url}/api/health"],
                    capture_output=True,
                    timeout=2
                )
                if response.returncode == 0:
                    return True
            except subprocess.TimeoutExpired:
                pass
            time.sleep(1)
        
        return False
    
    def _start_mock_firmware(self, scenario: str = "normal"):
        """Start the mock firmware node."""
        self.mock_firmware_process = subprocess.Popen(
            [
                "ros2", "launch", "sim", "mock_drive_test.launch.py",
                f"scenario:={scenario}"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        
        # Wait for mock firmware to start publishing
        time.sleep(3)
        return True
    
    async def _test_websocket_connection(self) -> Dict[str, Any]:
        """Test WebSocket connection and data reception."""
        received_data = []
        
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                self.websocket_connected = True
                
                # Collect data for a few seconds
                start_time = time.time()
                while time.time() - start_time < 5:
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                        data = json.loads(message)
                        received_data.append(data)
                    except asyncio.TimeoutError:
                        continue
                    except json.JSONDecodeError:
                        continue
                
        except Exception as e:
            self.fail(f"WebSocket connection failed: {e}")
        
        return received_data
    
    async def _test_rest_api(self) -> Dict[str, Any]:
        """Test REST API endpoints."""
        endpoints = [
            "/api/health",
            "/api/drive/summary",
            "/api/drive/diagnostics",
            "/api/drive/speeds",
            "/api/drive/status"
        ]
        
        results = {}
        
        async with aiohttp.ClientSession() as session:
            for endpoint in endpoints:
                try:
                    async with session.get(f"{self.ros_manager_url}{endpoint}") as response:
                        if response.status == 200:
                            data = await response.json()
                            results[endpoint] = data
                        else:
                            results[endpoint] = {"error": f"HTTP {response.status}"}
                except Exception as e:
                    results[endpoint] = {"error": str(e)}
        
        return results
    
    def test_basic_pipeline_functionality(self):
        """Test basic pipeline functionality with normal scenario."""
        # Start services
        self.assertTrue(self._start_ros_manager(), "Failed to start ROS Manager")
        self.assertTrue(self._start_mock_firmware("normal"), "Failed to start mock firmware")
        
        # Test REST API
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            api_results = loop.run_until_complete(self._test_rest_api())
            
            # Validate health endpoint
            self.assertIn("/api/health", api_results)
            health_data = api_results["/api/health"]
            self.assertNotIn("error", health_data)
            
            # Validate drive data endpoints
            for endpoint in ["/api/drive/summary", "/api/drive/diagnostics", "/api/drive/speeds"]:
                self.assertIn(endpoint, api_results)
                data = api_results[endpoint]
                self.assertNotIn("error", data, f"Error in {endpoint}: {data}")
            
            # Test WebSocket connection
            websocket_data = loop.run_until_complete(self._test_websocket_connection())
            self.assertTrue(len(websocket_data) > 0, "No data received via WebSocket")
            
            # Validate WebSocket data structure
            for data_point in websocket_data:
                self.assertIn("type", data_point)
                if data_point["type"] == "drive_data":
                    self.assertIn("data", data_point)
                    drive_data = data_point["data"]
                    self.assertIn("diagnostics", drive_data)
                    self.assertIn("speeds", drive_data)
                    
        finally:
            loop.close()
            self._stop_processes()
    
    def test_motor_fault_scenario(self):
        """Test motor fault scenario handling."""
        # Start services with motor fault scenario
        self.assertTrue(self._start_ros_manager(), "Failed to start ROS Manager")
        self.assertTrue(self._start_mock_firmware("motor_fault"), "Failed to start mock firmware")
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Wait for data to stabilize
            time.sleep(3)
            
            # Test motor status endpoint
            api_results = loop.run_until_complete(self._test_rest_api())
            
            status_data = api_results.get("/api/drive/status", {})
            self.assertNotIn("error", status_data)
            
            # In motor_fault scenario, RB motor should be disconnected
            if "data" in status_data:
                motor_status = status_data["data"]
                self.assertFalse(motor_status.get("RB", True), "RB motor should be disconnected in fault scenario")
            
        finally:
            loop.close()
            self._stop_processes()
    
    def test_data_accuracy(self):
        """Test data accuracy between ROS topics and API."""
        # This test would compare ROS topic data with API data
        # For now, we'll test that data is within expected ranges
        
        self.assertTrue(self._start_ros_manager(), "Failed to start ROS Manager")
        self.assertTrue(self._start_mock_firmware("normal"), "Failed to start mock firmware")
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            time.sleep(3)  # Wait for data
            
            api_results = loop.run_until_complete(self._test_rest_api())
            
            # Test diagnostics data ranges
            diagnostics = api_results.get("/api/drive/diagnostics", {}).get("data", {})
            
            for motor in ["RF", "RB", "LB", "LF"]:
                if motor in diagnostics:
                    motor_data = diagnostics[motor]
                    
                    # Voltage should be reasonable (9-15V)
                    voltage = motor_data.get("voltage", 0)
                    self.assertGreaterEqual(voltage, 9.0, f"{motor} voltage too low: {voltage}")
                    self.assertLessEqual(voltage, 15.0, f"{motor} voltage too high: {voltage}")
                    
                    # Current should be non-negative and reasonable (0-10A)
                    current = motor_data.get("current", 0)
                    self.assertGreaterEqual(current, 0.0, f"{motor} current negative: {current}")
                    self.assertLessEqual(current, 10.0, f"{motor} current too high: {current}")
                    
                    # Temperature should be reasonable (-20 to 100°C)
                    temperature = motor_data.get("temperature", 0)
                    self.assertGreaterEqual(temperature, -20.0, f"{motor} temperature too low: {temperature}")
                    self.assertLessEqual(temperature, 100.0, f"{motor} temperature too high: {temperature}")
            
        finally:
            loop.close()
            self._stop_processes()
    
    def test_connection_recovery(self):
        """Test connection recovery when services restart."""
        # Start initial services
        self.assertTrue(self._start_ros_manager(), "Failed to start ROS Manager")
        self.assertTrue(self._start_mock_firmware("normal"), "Failed to start mock firmware")
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Verify initial connection
            time.sleep(2)
            api_results = loop.run_until_complete(self._test_rest_api())
            self.assertNotIn("error", api_results["/api/health"])
            
            # Stop and restart mock firmware
            if self.mock_firmware_process:
                self.mock_firmware_process.terminate()
                self.mock_firmware_process.wait()
            
            time.sleep(2)  # Wait for disconnection
            
            # Restart mock firmware
            self.assertTrue(self._start_mock_firmware("normal"), "Failed to restart mock firmware")
            
            # Wait for reconnection
            time.sleep(3)
            
            # Verify connection is restored
            api_results = loop.run_until_complete(self._test_rest_api())
            self.assertNotIn("error", api_results["/api/health"])
            
        finally:
            loop.close()
            self._stop_processes()


class TestRunner:
    """Custom test runner for integration tests."""
    
    def __init__(self):
        self.results = []
    
    def run_tests(self, test_scenarios: List[str] = None):
        """Run integration tests with specified scenarios."""
        if test_scenarios is None:
            test_scenarios = ["normal", "motor_fault", "low_battery"]
        
        suite = unittest.TestSuite()
        
        # Add basic tests
        suite.addTest(ROSToUIPipelineTest('test_basic_pipeline_functionality'))
        suite.addTest(ROSToUIPipelineTest('test_data_accuracy'))
        suite.addTest(ROSToUIPipelineTest('test_connection_recovery'))
        
        # Add scenario-specific tests
        for scenario in test_scenarios:
            if scenario == "motor_fault":
                suite.addTest(ROSToUIPipelineTest('test_motor_fault_scenario'))
        
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        return result.wasSuccessful()


if __name__ == "__main__":
    # Check if ROS2 environment is available
    try:
        subprocess.run(["ros2", "--version"], check=True, capture_output=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: ROS2 not found. Please source your ROS2 environment.")
        sys.exit(1)
    
    # Run tests
    test_runner = TestRunner()
    success = test_runner.run_tests()
    
    sys.exit(0 if success else 1)
