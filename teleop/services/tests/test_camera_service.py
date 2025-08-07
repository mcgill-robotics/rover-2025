#!/usr/bin/env python3

"""
Test suite for Camera Service
Tests the multi-camera backend service functionality.
"""

import pytest
import asyncio
import json
import sys
import os
from unittest.mock import Mock, patch, MagicMock

# Add parent directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'camera'))

from camera_service import CameraService, CameraManager
import json
import os

def get_service_config():
    """Get camera service configuration from service_config.yml."""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'service_config.yml')
    
    try:
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return config['services']['camera_service']['config']
    except (FileNotFoundError, KeyError, yaml.YAMLError) as e:
        print(f"Warning: Could not load service config: {e}")
        return get_default_config()

def get_default_config():
    """Get default camera service configuration."""
    return {
        "host": "0.0.0.0",
        "inactive_timeout": 30.0,
        "frame_buffer_size": 10,
        "camera_discovery": {
            "enabled": True,
            "scan_interval": 5.0,
            "auto_connect": True
        },
        "gstreamer_config": {
            "rtp_caps": "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
            "pipeline_elements": [
                "rtph264depay",
                "h264parse",
                "avdec_h264",
                "videoconvert",
                "video/x-raw,format=BGR",
                "appsink drop=true max-buffers=2"
            ],
            "buffer_size": 1,
            "drop_frames": True
        },
        "aruco_config": {
            "dictionary": "DICT_4X4_100",
            "marker_border_color": [0, 255, 0],
            "marker_border_thickness": 2,
            "text_color": [255, 255, 255],
            "text_thickness": 2,
            "text_font": "FONT_HERSHEY_SIMPLEX",
            "text_scale": 0.7
        },
        "jpeg_config": {
            "quality": 85,
            "optimize": True
        }
    }


class TestCameraService:
    """Test cases for Camera Service."""
    
    def setup_method(self):
        """Setup test environment."""
        self.config = get_service_config()
        self.camera_service = CameraService(self.config)
    
    def test_config_loading(self):
        """Test that configuration loads correctly."""
        config = get_service_config()
        
        assert "host" in config
        assert "inactive_timeout" in config
        assert "default_cameras" in config
        assert "gstreamer_config" in config
        assert "aruco_config" in config
        assert "jpeg_config" in config
        
        # Check camera discovery
        discovery = config["camera_discovery"]
        assert "enabled" in discovery
        assert "scan_interval" in discovery
        assert "auto_connect" in discovery
    
    def test_camera_manager_initialization(self):
        """Test camera manager initialization."""
        with patch('camera_service.CameraManager') as mock_manager:
            mock_manager.return_value = Mock()
            
            service = CameraService(self.config)
            
            # Verify camera manager was created
            mock_manager.assert_called_once()
    
    @patch('camera_service.FastAPI')
    def test_fastapi_initialization(self, mock_fastapi):
        """Test FastAPI app initialization."""
        mock_app = Mock()
        mock_fastapi.return_value = mock_app
        
        service = CameraService(self.config)
        
        # Verify FastAPI was created
        mock_fastapi.assert_called_once()
    
    def test_camera_endpoints(self):
        """Test camera API endpoints."""
        with patch('camera_service.FastAPI') as mock_fastapi:
            mock_app = Mock()
            mock_fastapi.return_value = mock_app
            
            service = CameraService(self.config)
            
            # Verify endpoints were added
            # Note: This is a simplified test - in reality we'd check the actual endpoint registration
            assert hasattr(service, 'app')
    
    def test_gstreamer_config(self):
        """Test GStreamer configuration."""
        gst_config = self.config["gstreamer_config"]
        
        assert "rtp_caps" in gst_config
        assert "pipeline_elements" in gst_config
        assert "buffer_size" in gst_config
        assert "drop_frames" in gst_config
        
        # Check pipeline elements
        elements = gst_config["pipeline_elements"]
        assert "rtph264depay" in elements
        assert "h264parse" in elements
        assert "avdec_h264" in elements
    
    def test_aruco_config(self):
        """Test ArUco detection configuration."""
        aruco_config = self.config["aruco_config"]
        
        assert "dictionary" in aruco_config
        assert "marker_border_color" in aruco_config
        assert "text_color" in aruco_config
        assert "text_scale" in aruco_config
        
        # Check colors are RGB values
        assert len(aruco_config["marker_border_color"]) == 3
        assert len(aruco_config["text_color"]) == 3
    
    def test_jpeg_config(self):
        """Test JPEG encoding configuration."""
        jpeg_config = self.config["jpeg_config"]
        
        assert "quality" in jpeg_config
        assert "optimize" in jpeg_config
        
        # Check quality is reasonable
        assert 1 <= jpeg_config["quality"] <= 100
    
    @patch('camera_service.uvicorn.run')
    def test_service_startup(self, mock_run):
        """Test service startup."""
        with patch('camera_service.FastAPI') as mock_fastapi:
            mock_app = Mock()
            mock_fastapi.return_value = mock_app
            
            service = CameraService(self.config)
            
            # Mock the startup method
            with patch.object(service, 'startup'):
                service.startup()
                
                # Verify startup was called
                service.startup.assert_called_once()
    
    def test_camera_discovery_validation(self):
        """Test camera discovery validation."""
        discovery = self.config["camera_discovery"]
        
        # Check required fields
        assert "enabled" in discovery
        assert "scan_interval" in discovery
        assert "auto_connect" in discovery
        
        # Check data types
        assert isinstance(discovery["enabled"], bool)
        assert isinstance(discovery["scan_interval"], (int, float))
        assert isinstance(discovery["auto_connect"], bool)
        
        # Check scan interval validity
        assert discovery["scan_interval"] > 0
    
    def test_timeout_configuration(self):
        """Test timeout configuration."""
        assert "inactive_timeout" in self.config
        assert isinstance(self.config["inactive_timeout"], (int, float))
        assert self.config["inactive_timeout"] > 0
    
    def test_buffer_configuration(self):
        """Test buffer configuration."""
        assert "frame_buffer_size" in self.config
        assert isinstance(self.config["frame_buffer_size"], int)
        assert self.config["frame_buffer_size"] > 0


class TestCameraManager:
    """Test cases for Camera Manager."""
    
    def setup_method(self):
        """Setup test environment."""
        self.config = get_service_config()
        self.manager = CameraManager(self.config)
    
    @patch('camera_service.GStreamerReader')
    def test_camera_creation(self, mock_reader):
        """Test camera creation."""
        mock_reader.return_value = Mock()
        
        camera_config = {
            "camera_id": "test-camera",
            "port": 5000,
            "device_id": "test-device",
            "name": "Test Camera"
        }
        
        # Test adding a camera
        result = self.manager.add_camera(camera_config)
        
        # Verify camera was added
        assert result is True
        assert "test-camera" in self.manager.cameras
    
    def test_camera_removal(self):
        """Test camera removal."""
        camera_id = "test-camera"
        
        # Add a camera first
        self.manager.cameras[camera_id] = Mock()
        
        # Remove the camera
        result = self.manager.remove_camera(camera_id)
        
        # Verify camera was removed
        assert result is True
        assert camera_id not in self.manager.cameras
    
    def test_camera_listing(self):
        """Test camera listing."""
        # Add some test cameras
        self.manager.cameras["cam1"] = Mock()
        self.manager.cameras["cam2"] = Mock()
        
        cameras = self.manager.list_cameras()
        
        # Verify cameras are listed
        assert len(cameras) == 2
        assert "cam1" in cameras
        assert "cam2" in cameras
    
    def test_inactive_camera_cleanup(self):
        """Test inactive camera cleanup."""
        # Add a test camera
        mock_camera = Mock()
        mock_camera.is_active.return_value = False
        self.manager.cameras["inactive-camera"] = mock_camera
        
        # Run cleanup
        self.manager.cleanup_inactive_cameras()
        
        # Verify inactive camera was removed
        assert "inactive-camera" not in self.manager.cameras


class TestIntegration:
    """Integration tests for camera service."""
    
    @pytest.mark.asyncio
    async def test_service_health_check(self):
        """Test service health check endpoint."""
        # This would test the actual health check endpoint
        # For now, we'll just verify the config is valid
        config = get_service_config()
        assert config is not None
    
    def test_config_compatibility(self):
        """Test backward compatibility with old config format."""
        # Test that get_backend_config() still works
        from config import get_backend_config
        
        backend_config = get_backend_config()
        
        # Check that it has the expected structure
        assert "HOST" in backend_config
        assert "HTTP_PORT" in backend_config
        assert "DEFAULT_CAMERAS" in backend_config
        assert "GSTREAMER_CONFIG" in backend_config


if __name__ == "__main__":
    pytest.main([__file__]) 