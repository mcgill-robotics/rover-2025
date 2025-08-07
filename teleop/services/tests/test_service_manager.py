#!/usr/bin/env python3

"""
Test suite for the Unified Service Manager
"""

import asyncio
import json
import pytest
import sys
import os
import time
from unittest.mock import Mock, patch, AsyncMock
from typing import Dict, Any

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from service_manager import ServiceManager, ServiceManagerAPI


class TestServiceManager:
    """Test cases for ServiceManager class."""
    
    @pytest.fixture
    def service_manager(self):
        """Create a service manager instance for testing."""
        config = {
            'services': {
                'ros_manager': {
                    'port': 8082,
                    'enabled': True,
                    'health_check_url': '/api/health'
                },
                'gps_service': {
                    'port': 5001,
                    'enabled': True,
                    'health_check_url': '/api/health'
                }
            },
            'monitoring': {
                'health_check_interval': 1,  # Short interval for testing
                'restart_failed_services': True,
                'max_restart_attempts': 3
            }
        }
        return ServiceManager(config)
    
    def test_default_config(self):
        """Test default configuration generation."""
        manager = ServiceManager()
        config = manager.config
        
        assert 'services' in config
        assert 'ros_manager' in config['services']
        assert 'gps_service' in config['services']
        assert 'monitoring' in config
        assert 'logging' in config
    
    def test_custom_config(self):
        """Test custom configuration loading."""
        custom_config = {
            'services': {
                'test_service': {
                    'port': 9999,
                    'enabled': True
                }
            }
        }
        manager = ServiceManager(custom_config)
        
        assert manager.config['services']['test_service']['port'] == 9999
        assert manager.config['services']['test_service']['enabled'] is True
    
    @pytest.mark.asyncio
    async def test_initialization(self, service_manager):
        """Test service manager initialization."""
        # Mock ROS Manager and GPS service to avoid actual initialization
        with patch('service_manager.ROSBridge') as mock_ros_bridge, \
             patch('service_manager.start_gps_service') as mock_gps_start:
            
            mock_ros_bridge.return_value.initialize = AsyncMock()
            mock_gps_start.return_value = None
            
            await service_manager.initialize()
            
            assert service_manager.is_running is True
            assert 'ros_manager' in service_manager.services
            assert 'gps_service' in service_manager.services
    
    @pytest.mark.asyncio
    async def test_health_monitoring(self, service_manager):
        """Test health monitoring functionality."""
        # Mock aiohttp client session
        with patch('aiohttp.ClientSession') as mock_session:
            mock_response = Mock()
            mock_response.status = 200
            mock_response.headers.get.return_value = '100ms'
            
            mock_session.return_value.__aenter__.return_value.get.return_value.__aenter__.return_value = mock_response
            
            # Start health monitoring
            service_manager.is_running = True
            await service_manager._check_service_health()
            
            # Check that health status was updated
            assert 'ros_manager' in service_manager.health_status
            assert service_manager.health_status['ros_manager']['status'] == 'healthy'
    
    @pytest.mark.asyncio
    async def test_service_status(self, service_manager):
        """Test service status reporting."""
        service_manager.is_running = True
        service_manager._start_time = time.time()
        service_manager.health_status = {
            'ros_manager': {'status': 'healthy'},
            'gps_service': {'status': 'error'}
        }
        
        status = await service_manager.get_service_status()
        
        assert status['manager_status'] == 'running'
        assert 'services' in status
        assert 'uptime' in status
        assert 'config' in status
        assert status['services']['ros_manager']['status'] == 'healthy'
        assert status['services']['gps_service']['status'] == 'error'
    
    @pytest.mark.asyncio
    async def test_service_restart(self, service_manager):
        """Test service restart functionality."""
        # Mock ROS Manager
        mock_ros_manager = Mock()
        mock_ros_manager.shutdown = AsyncMock()
        service_manager.ros_manager = mock_ros_manager
        service_manager.services['ros_manager'] = mock_ros_manager
        
        with patch('service_manager.ROSBridge') as mock_ros_bridge:
            mock_ros_bridge.return_value.initialize = AsyncMock()
            
            success = await service_manager.restart_service('ros_manager')
            
            assert success is True
            mock_ros_manager.shutdown.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_shutdown(self, service_manager):
        """Test graceful shutdown."""
        # Mock ROS Manager
        mock_ros_manager = Mock()
        mock_ros_manager.shutdown = AsyncMock()
        service_manager.ros_manager = mock_ros_manager
        
        await service_manager.shutdown()
        
        assert service_manager.is_running is False
        mock_ros_manager.shutdown.assert_called_once()


class TestServiceManagerAPI:
    """Test cases for ServiceManagerAPI class."""
    
    @pytest.fixture
    def api_server(self):
        """Create an API server instance for testing."""
        mock_service_manager = Mock()
        mock_service_manager.get_service_status = AsyncMock()
        mock_service_manager.restart_service = AsyncMock()
        mock_service_manager.config = {'test': 'config'}
        mock_service_manager.is_running = True
        
        return ServiceManagerAPI(mock_service_manager, port=8083)
    
    @pytest.mark.asyncio
    async def test_get_status(self, api_server):
        """Test status endpoint."""
        mock_request = Mock()
        mock_response = {'status': 'test'}
        api_server.service_manager.get_service_status.return_value = mock_response
        
        response = await api_server.get_status(mock_request)
        
        assert response.status == 200
        api_server.service_manager.get_service_status.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_restart_service(self, api_server):
        """Test service restart endpoint."""
        mock_request = Mock()
        mock_request.match_info = {'service': 'test_service'}
        api_server.service_manager.restart_service.return_value = True
        
        response = await api_server.restart_service(mock_request)
        
        assert response.status == 200
        api_server.service_manager.restart_service.assert_called_once_with('test_service')
    
    @pytest.mark.asyncio
    async def test_get_config(self, api_server):
        """Test config endpoint."""
        mock_request = Mock()
        
        response = await api_server.get_config(mock_request)
        
        assert response.status == 200
    
    @pytest.mark.asyncio
    async def test_health_check(self, api_server):
        """Test health check endpoint."""
        mock_request = Mock()
        
        response = await api_server.health_check(mock_request)
        
        assert response.status == 200


class TestIntegration:
    """Integration tests for the service manager."""
    
    @pytest.mark.asyncio
    async def test_full_initialization_cycle(self):
        """Test complete initialization and shutdown cycle."""
        config = {
            'services': {
                'ros_manager': {
                    'port': 8082,
                    'enabled': False,  # Disable for testing
                    'health_check_url': '/api/health'
                },
                'gps_service': {
                    'port': 5001,
                    'enabled': False,  # Disable for testing
                    'health_check_url': '/api/health'
                }
            }
        }
        
        manager = ServiceManager(config)
        
        # Test initialization
        await manager.initialize()
        assert manager.is_running is True
        
        # Test status reporting
        status = await manager.get_service_status()
        assert status['manager_status'] == 'running'
        
        # Test shutdown
        await manager.shutdown()
        assert manager.is_running is False


def run_tests():
    """Run all tests."""
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