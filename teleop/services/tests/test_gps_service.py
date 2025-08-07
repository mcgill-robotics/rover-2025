#!/usr/bin/env python3

"""
Test suite for the GPS Service
"""

import pytest
import sys
import os
import json
import time
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, Any

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

# Mock ROS imports for testing
sys.modules['rclpy'] = Mock()
sys.modules['sensor_msgs.msg'] = Mock()
sys.modules['geometry_msgs.msg'] = Mock()

from gps.gps_service import GPSService


class TestGPSService:
    """Test cases for GPSService class."""
    
    @pytest.fixture
    def gps_service(self):
        """Create a GPS service instance for testing."""
        with patch('gps.gps_service.rclpy'):
            return GPSService()
    
    def test_initialization(self, gps_service):
        """Test GPS service initialization."""
        assert gps_service.gps_data is not None
        assert 'latitude' in gps_service.gps_data
        assert 'longitude' in gps_service.gps_data
        assert 'heading' in gps_service.gps_data
        assert 'accuracy' in gps_service.gps_data
        assert 'timestamp' in gps_service.gps_data
    
    def test_gps_callback(self, gps_service):
        """Test GPS callback processing."""
        # Create mock GPS message
        mock_msg = Mock()
        mock_msg.latitude = 45.5048
        mock_msg.longitude = -73.5772
        mock_msg.status.status = 1
        mock_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        mock_msg.position_covariance_type = 1
        
        # Call the callback
        gps_service.gps_callback(mock_msg)
        
        # Check that data was updated
        assert gps_service.gps_data['latitude'] == 45.5048
        assert gps_service.gps_data['longitude'] == -73.5772
        assert gps_service.gps_data['fix_quality'] == 1
        assert gps_service.gps_data['timestamp'] > 0
    
    def test_imu_callback(self, gps_service):
        """Test IMU callback processing."""
        # Create mock IMU message
        mock_msg = Mock()
        mock_msg.orientation.x = 0.0
        mock_msg.orientation.y = 0.0
        mock_msg.orientation.z = 0.707  # sin(45°)
        mock_msg.orientation.w = 0.707  # cos(45°)
        
        # Call the callback
        gps_service.imu_callback(mock_msg)
        
        # Check that heading was updated (should be around 45 degrees)
        heading = gps_service.gps_data['heading']
        assert 40 <= heading <= 50  # Allow some tolerance
    
    def test_velocity_callback(self, gps_service):
        """Test velocity callback processing."""
        # Create mock velocity message
        mock_msg = Mock()
        mock_msg.twist.linear.x = 1.0
        mock_msg.twist.linear.y = 0.0
        
        # Call the callback
        gps_service.velocity_callback(mock_msg)
        
        # Check that heading was updated (should be around 0 degrees)
        heading = gps_service.gps_data['heading']
        assert 0 <= heading <= 10  # Allow some tolerance
    
    def test_accuracy_calculation(self, gps_service):
        """Test GPS accuracy calculation."""
        # Test with known covariance
        mock_msg = Mock()
        mock_msg.position_covariance_type = 1
        mock_msg.position_covariance = [4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 1.0]
        
        accuracy = gps_service.calculate_accuracy(mock_msg)
        
        # Should be 2 * sqrt(max(4, 9)) = 2 * 3 = 6
        assert accuracy == 6.0
    
    def test_accuracy_calculation_unknown(self, gps_service):
        """Test GPS accuracy calculation with unknown covariance."""
        mock_msg = Mock()
        mock_msg.position_covariance_type = 0  # Unknown
        
        accuracy = gps_service.calculate_accuracy(mock_msg)
        
        # Should return default accuracy
        assert accuracy == 50.0
    
    def test_get_gps_data(self, gps_service):
        """Test GPS data retrieval."""
        # Set some test data
        gps_service.gps_data = {
            'latitude': 45.5048,
            'longitude': -73.5772,
            'heading': 180.0,
            'accuracy': 5.0,
            'timestamp': 1234567890,
            'fix_quality': 1,
            'satellites': 8
        }
        
        data = gps_service.get_gps_data()
        
        assert data['latitude'] == 45.5048
        assert data['longitude'] == -73.5772
        assert data['heading'] == 180.0
        assert data['accuracy'] == 5.0
        assert data['timestamp'] == 1234567890
        assert data['fix_quality'] == 1
        assert data['satellites'] == 8
    
    def test_get_gps_status(self, gps_service):
        """Test GPS status retrieval."""
        # Set some test data
        gps_service.gps_data = {
            'latitude': 45.5048,
            'longitude': -73.5772,
            'heading': 180.0,
            'accuracy': 5.0,
            'timestamp': 1234567890,
            'fix_quality': 1,
            'satellites': 8
        }
        
        status = gps_service.get_gps_status()
        
        assert status['has_fix'] is True
        assert status['fix_quality'] == 1
        assert status['satellites'] == 8
        assert status['accuracy'] == 5.0
        assert status['last_update'] == 1234567890
    
    def test_get_gps_status_no_fix(self, gps_service):
        """Test GPS status when no fix is available."""
        # Set data with no fix
        gps_service.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'heading': 0.0,
            'accuracy': 0.0,
            'timestamp': 1234567890,
            'fix_quality': 0,
            'satellites': 0
        }
        
        status = gps_service.get_gps_status()
        
        assert status['has_fix'] is False
        assert status['fix_quality'] == 0
        assert status['satellites'] == 0


class TestGPSAPIIntegration:
    """Integration tests for GPS API."""
    
    @pytest.fixture
    def gps_api(self):
        """Create a GPS API instance for testing."""
        with patch('gps.gps_api.rclpy'):
            from gps.gps_api import app
            return app.test_client()
    
    def test_gps_data_endpoint(self, gps_api):
        """Test GPS data endpoint."""
        response = gps_api.get('/api/gps/data')
        
        assert response.status_code == 200
        data = json.loads(response.data)
        
        # Should return GPS data structure
        assert 'latitude' in data
        assert 'longitude' in data
        assert 'heading' in data
        assert 'accuracy' in data
        assert 'timestamp' in data
    
    def test_gps_status_endpoint(self, gps_api):
        """Test GPS status endpoint."""
        response = gps_api.get('/api/gps/status')
        
        assert response.status_code == 200
        data = json.loads(response.data)
        
        # Should return status structure
        assert 'has_fix' in data
        assert 'fix_quality' in data
        assert 'satellites' in data
        assert 'accuracy' in data
        assert 'last_update' in data
    
    def test_health_check_endpoint(self, gps_api):
        """Test health check endpoint."""
        response = gps_api.get('/api/health')
        
        assert response.status_code == 200
        data = json.loads(response.data)
        
        assert 'status' in data
        assert data['status'] == 'healthy'
    
    def test_config_endpoint(self, gps_api):
        """Test config endpoint."""
        response = gps_api.get('/api/config')
        
        assert response.status_code == 200
        data = json.loads(response.data)
        
        # Should return configuration
        assert 'gps_topic' in data
        assert 'imu_topic' in data
        assert 'velocity_topic' in data


class TestMockGPSData:
    """Tests for mock GPS data generation."""
    
    def test_mock_gps_data_generation(self):
        """Test that mock GPS data is generated correctly."""
        with patch('gps.gps_service.rclpy'):
            service = GPSService()
            
            # Simulate mock data generation
            service.gps_data = {
                'latitude': 45.5048,
                'longitude': -73.5772,
                'heading': 180.0,
                'accuracy': 5.0,
                'timestamp': time.time() * 1000,
                'fix_quality': 1,
                'satellites': 8
            }
            
            data = service.get_gps_data()
            
            # Check data structure
            assert isinstance(data['latitude'], float)
            assert isinstance(data['longitude'], float)
            assert isinstance(data['heading'], float)
            assert isinstance(data['accuracy'], float)
            assert isinstance(data['timestamp'], (int, float))
            assert isinstance(data['fix_quality'], int)
            assert isinstance(data['satellites'], int)
            
            # Check reasonable ranges
            assert -90 <= data['latitude'] <= 90
            assert -180 <= data['longitude'] <= 180
            assert 0 <= data['heading'] <= 360
            assert data['accuracy'] >= 0
            assert data['timestamp'] > 0


def run_tests():
    """Run all GPS service tests."""
    import pytest
    
    # Run tests with verbose output
    pytest.main([
        __file__,
        '-v',
        '--tb=short',
        '--strict-markers',
        '--disable-warnings'
    ])


if __name__ == '__main__':
    run_tests() 