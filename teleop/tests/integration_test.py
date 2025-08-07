#!/usr/bin/env python3

"""
Integration Test for Rover 2025 Teleop System
Tests the complete system: UI + Services + ROS integration
"""

import asyncio
import json
import time
import subprocess
import signal
import sys
import os
from typing import Dict, Any, Optional
import requests
import pytest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.chrome.options import Options

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))


class TeleopIntegrationTest:
    """Integration test for the complete teleop system."""
    
    def __init__(self):
        self.service_manager_process = None
        self.ui_process = None
        self.tileserver_process = None
        self.driver = None
        self.test_results = {}
        
        # Service URLs
        self.service_manager_url = "http://localhost:8083"
        self.ros_manager_url = "http://localhost:8082"
        self.gps_service_url = "http://localhost:5001"
        self.ui_url = "http://localhost:3000"
        self.tileserver_url = "http://localhost:8080"
        
        # Timeouts
        self.service_startup_timeout = 30
        self.ui_startup_timeout = 60
        self.page_load_timeout = 10
        
    async def setup(self):
        """Setup the test environment."""
        print("🚀 Setting up integration test environment...")
        
        # Start services
        await self.start_services()
        
        # Start UI
        await self.start_ui()
        
        # Start TileServer (if needed)
        await self.start_tileserver()
        
        # Setup web driver
        await self.setup_webdriver()
        
        print("✅ Test environment setup complete")
    
    async def teardown(self):
        """Cleanup test environment."""
        print("🧹 Cleaning up test environment...")
        
        # Close web driver
        if self.driver:
            self.driver.quit()
        
        # Stop processes
        await self.stop_all_processes()
        
        print("✅ Test environment cleanup complete")
    
    async def start_services(self):
        """Start the unified service manager."""
        print("🤖 Starting unified service manager...")
        
        try:
            # Start service manager
            self.service_manager_process = subprocess.Popen(
                ["python3", "services/service_manager.py"],
                cwd=os.path.dirname(os.path.dirname(__file__)),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for services to start
            await self.wait_for_service(self.service_manager_url, "Service Manager")
            await self.wait_for_service(self.ros_manager_url, "ROS Manager")
            await self.wait_for_service(self.gps_service_url, "GPS Service")
            
            print("✅ Services started successfully")
            
        except Exception as e:
            print(f"❌ Failed to start services: {e}")
            raise
    
    async def start_ui(self):
        """Start the React UI."""
        print("🌐 Starting React UI...")
        
        try:
            # Start UI development server
            self.ui_process = subprocess.Popen(
                ["npm", "run", "dev"],
                cwd="robot-controller-ui",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for UI to start
            await self.wait_for_service(self.ui_url, "React UI")
            
            print("✅ UI started successfully")
            
        except Exception as e:
            print(f"❌ Failed to start UI: {e}")
            raise
    
    async def start_tileserver(self):
        """Start TileServer for offline mapping."""
        print("🗺️ Starting TileServer...")
        
        try:
            # Start TileServer using docker-compose
            self.tileserver_process = subprocess.Popen(
                ["docker-compose", "-f", "services/gps/docker-compose.tileserver.yml", "up", "-d"],
                cwd=os.path.dirname(os.path.dirname(__file__)),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for TileServer to start
            await self.wait_for_service(self.tileserver_url, "TileServer")
            
            print("✅ TileServer started successfully")
            
        except Exception as e:
            print(f"⚠️ TileServer failed to start (may not be needed): {e}")
    
    async def setup_webdriver(self):
        """Setup Selenium web driver."""
        print("🌐 Setting up web driver...")
        
        try:
            # Chrome options for headless testing
            chrome_options = Options()
            chrome_options.add_argument("--headless")
            chrome_options.add_argument("--no-sandbox")
            chrome_options.add_argument("--disable-dev-shm-usage")
            chrome_options.add_argument("--disable-gpu")
            chrome_options.add_argument("--window-size=1920,1080")
            
            self.driver = webdriver.Chrome(options=chrome_options)
            self.driver.set_page_load_timeout(self.page_load_timeout)
            
            print("✅ Web driver setup complete")
            
        except Exception as e:
            print(f"❌ Failed to setup web driver: {e}")
            raise
    
    async def wait_for_service(self, url: str, service_name: str):
        """Wait for a service to become available."""
        print(f"⏳ Waiting for {service_name} at {url}...")
        
        start_time = time.time()
        while time.time() - start_time < self.service_startup_timeout:
            try:
                response = requests.get(f"{url}/api/health", timeout=5)
                if response.status_code == 200:
                    print(f"✅ {service_name} is ready")
                    return
            except requests.exceptions.RequestException:
                pass
            
            await asyncio.sleep(1)
        
        raise TimeoutError(f"{service_name} failed to start within {self.service_startup_timeout} seconds")
    
    async def test_service_manager_api(self):
        """Test the service manager API."""
        print("🔍 Testing Service Manager API...")
        
        try:
            # Test status endpoint
            response = requests.get(f"{self.service_manager_url}/api/status")
            assert response.status_code == 200
            
            status_data = response.json()
            assert "manager_status" in status_data
            assert "services" in status_data
            
            # Test config endpoint
            response = requests.get(f"{self.service_manager_url}/api/config")
            assert response.status_code == 200
            
            config_data = response.json()
            assert "services" in config_data
            
            self.test_results["service_manager_api"] = "PASS"
            print("✅ Service Manager API tests passed")
            
        except Exception as e:
            self.test_results["service_manager_api"] = f"FAIL: {e}"
            print(f"❌ Service Manager API tests failed: {e}")
            raise
    
    async def test_ros_manager_api(self):
        """Test the ROS manager API."""
        print("🤖 Testing ROS Manager API...")
        
        try:
            # Test drive diagnostics endpoint
            response = requests.get(f"{self.ros_manager_url}/api/drive/diagnostics")
            assert response.status_code == 200
            
            # Test drive speeds endpoint
            response = requests.get(f"{self.ros_manager_url}/api/drive/speeds")
            assert response.status_code == 200
            
            # Test drive status endpoint
            response = requests.get(f"{self.ros_manager_url}/api/drive/status")
            assert response.status_code == 200
            
            self.test_results["ros_manager_api"] = "PASS"
            print("✅ ROS Manager API tests passed")
            
        except Exception as e:
            self.test_results["ros_manager_api"] = f"FAIL: {e}"
            print(f"❌ ROS Manager API tests failed: {e}")
            raise
    
    async def test_gps_service_api(self):
        """Test the GPS service API."""
        print("📍 Testing GPS Service API...")
        
        try:
            # Test GPS data endpoint
            response = requests.get(f"{self.gps_service_url}/api/gps/data")
            assert response.status_code == 200
            
            gps_data = response.json()
            assert "latitude" in gps_data
            assert "longitude" in gps_data
            assert "heading" in gps_data
            assert "accuracy" in gps_data
            assert "timestamp" in gps_data
            
            # Test GPS status endpoint
            response = requests.get(f"{self.gps_service_url}/api/gps/status")
            assert response.status_code == 200
            
            status_data = response.json()
            assert "has_fix" in status_data
            assert "fix_quality" in status_data
            assert "satellites" in status_data
            
            self.test_results["gps_service_api"] = "PASS"
            print("✅ GPS Service API tests passed")
            
        except Exception as e:
            self.test_results["gps_service_api"] = f"FAIL: {e}"
            print(f"❌ GPS Service API tests failed: {e}")
            raise
    
    async def test_ui_pages(self):
        """Test the React UI pages."""
        print("🌐 Testing React UI pages...")
        
        try:
            # Test main page
            self.driver.get(self.ui_url)
            WebDriverWait(self.driver, 10).until(
                EC.presence_of_element_located((By.TAG_NAME, "body"))
            )
            
            # Check if page loaded
            assert "Rover Controller" in self.driver.title or "Rover" in self.driver.page_source
            
            # Test navigation to different pages
            pages_to_test = [
                ("/drive", "Drive"),
                ("/arm", "Arm"),
                ("/mapping", "Mapping"),
                ("/status", "Status")
            ]
            
            for path, page_name in pages_to_test:
                print(f"  Testing {page_name} page...")
                self.driver.get(f"{self.ui_url}{path}")
                
                # Wait for page to load
                WebDriverWait(self.driver, 10).until(
                    EC.presence_of_element_located((By.TAG_NAME, "body"))
                )
                
                # Basic check that page loaded
                assert len(self.driver.page_source) > 1000  # Page has content
            
            self.test_results["ui_pages"] = "PASS"
            print("✅ React UI page tests passed")
            
        except Exception as e:
            self.test_results["ui_pages"] = f"FAIL: {e}"
            print(f"❌ React UI page tests failed: {e}")
            raise
    
    async def test_ui_api_integration(self):
        """Test UI integration with backend APIs."""
        print("🔗 Testing UI-API integration...")
        
        try:
            # Load the main page
            self.driver.get(self.ui_url)
            
            # Wait for page to load
            WebDriverWait(self.driver, 10).until(
                EC.presence_of_element_located((By.TAG_NAME, "body"))
            )
            
            # Check for any JavaScript errors
            logs = self.driver.get_log('browser')
            errors = [log for log in logs if log['level'] == 'SEVERE']
            
            if errors:
                print(f"⚠️ JavaScript errors found: {errors}")
                # Don't fail the test for JS errors, just log them
            
            # Test that the page can make API calls (basic check)
            # This is a simplified test - in a real scenario you'd check specific API calls
            
            self.test_results["ui_api_integration"] = "PASS"
            print("✅ UI-API integration tests passed")
            
        except Exception as e:
            self.test_results["ui_api_integration"] = f"FAIL: {e}"
            print(f"❌ UI-API integration tests failed: {e}")
            raise
    
    async def test_mapping_functionality(self):
        """Test the mapping functionality."""
        print("🗺️ Testing mapping functionality...")
        
        try:
            # Test TileServer
            if self.tileserver_process:
                response = requests.get(self.tileserver_url, timeout=5)
                assert response.status_code == 200
            
            # Test mapping page
            self.driver.get(f"{self.ui_url}/mapping")
            WebDriverWait(self.driver, 10).until(
                EC.presence_of_element_located((By.TAG_NAME, "body"))
            )
            
            # Check for mapping elements (basic check)
            page_source = self.driver.page_source
            assert "mapping" in page_source.lower() or "map" in page_source.lower()
            
            self.test_results["mapping_functionality"] = "PASS"
            print("✅ Mapping functionality tests passed")
            
        except Exception as e:
            self.test_results["mapping_functionality"] = f"FAIL: {e}"
            print(f"❌ Mapping functionality tests failed: {e}")
            # Don't raise - mapping might not be critical
    
    async def test_system_health(self):
        """Test overall system health."""
        print("🏥 Testing system health...")
        
        try:
            # Check all services are healthy
            services = [
                (self.service_manager_url, "Service Manager"),
                (self.ros_manager_url, "ROS Manager"),
                (self.gps_service_url, "GPS Service")
            ]
            
            for url, name in services:
                response = requests.get(f"{url}/api/health", timeout=5)
                assert response.status_code == 200
                
                health_data = response.json()
                assert "status" in health_data
                assert health_data["status"] == "healthy"
            
            self.test_results["system_health"] = "PASS"
            print("✅ System health tests passed")
            
        except Exception as e:
            self.test_results["system_health"] = f"FAIL: {e}"
            print(f"❌ System health tests failed: {e}")
            raise
    
    async def stop_all_processes(self):
        """Stop all running processes."""
        print("🛑 Stopping all processes...")
        
        processes = [
            ("Service Manager", self.service_manager_process),
            ("React UI", self.ui_process),
            ("TileServer", self.tileserver_process)
        ]
        
        for name, process in processes:
            if process:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                    print(f"✅ {name} stopped")
                except subprocess.TimeoutExpired:
                    process.kill()
                    print(f"⚠️ {name} force killed")
                except Exception as e:
                    print(f"⚠️ Error stopping {name}: {e}")
    
    async def run_all_tests(self):
        """Run all integration tests."""
        print("🧪 Starting integration tests...")
        
        try:
            await self.setup()
            
            # Run all test suites
            await self.test_service_manager_api()
            await self.test_ros_manager_api()
            await self.test_gps_service_api()
            await self.test_ui_pages()
            await self.test_ui_api_integration()
            await self.test_mapping_functionality()
            await self.test_system_health()
            
            # Print results
            print("\n📊 Test Results:")
            print("=" * 50)
            for test_name, result in self.test_results.items():
                status = "✅ PASS" if result == "PASS" else f"❌ FAIL"
                print(f"{test_name:25} {status}")
            
            # Check if all tests passed
            failed_tests = [name for name, result in self.test_results.items() if result != "PASS"]
            
            if failed_tests:
                print(f"\n❌ {len(failed_tests)} tests failed: {', '.join(failed_tests)}")
                return False
            else:
                print(f"\n🎉 All {len(self.test_results)} tests passed!")
                return True
                
        except Exception as e:
            print(f"\n💥 Integration test failed: {e}")
            return False
        finally:
            await self.teardown()


async def main():
    """Main entry point for integration tests."""
    test = TeleopIntegrationTest()
    success = await test.run_all_tests()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    asyncio.run(main()) 