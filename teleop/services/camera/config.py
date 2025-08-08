#!/usr/bin/env python3
"""
Configuration loader for camera services.
Loads settings from service_config.yml.
"""

import os
import yaml
import logging

logger = logging.getLogger(__name__)

def get_backend_config():
    """Get backend configuration from service_config.yml."""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'service_config.yml')
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        camera_config = config['services']['camera_service']['config']
        
        # Convert to old format for compatibility
        return {
            "HOST": camera_config["host"],
            "HTTP_PORT": 8001,  # This will be overridden by service manager
            "UDP_PORT": 9999,
            "INACTIVE_TIMEOUT": camera_config["inactive_timeout"],
            "HEARTBEAT_TIMEOUT": 15.0,
            "FRAME_BUFFER_SIZE": camera_config["frame_buffer_size"],
            "MAX_UDP_PACKET_SIZE": 65507,
            "CAMERA_DISCOVERY": camera_config["camera_discovery"],
            "GSTREAMER_CONFIG": camera_config["gstreamer_config"],
            "ARUCO_CONFIG": camera_config["aruco_config"],
            "JPEG_CONFIG": camera_config["jpeg_config"]
        }
    except (FileNotFoundError, KeyError, yaml.YAMLError) as e:
        logger.warning(f"Could not load service config: {e}")
        # Return default config
        return {
            "HOST": "0.0.0.0",
            "HTTP_PORT": 8001,
            "UDP_PORT": 9999,
            "INACTIVE_TIMEOUT": 30.0,
            "HEARTBEAT_TIMEOUT": 15.0,
            "FRAME_BUFFER_SIZE": 10,
            "MAX_UDP_PACKET_SIZE": 65507,
            "CAMERA_DISCOVERY": {"enabled": True, "scan_interval": 5.0, "auto_connect": True},
            "GSTREAMER_CONFIG": {
                "rtp_caps": "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
                "pipeline_elements": ["rtph264depay", "h264parse", "avdec_h264", "videoconvert", "video/x-raw,format=BGR", "appsink drop=true max-buffers=2"],
                "buffer_size": 1,
                "drop_frames": True
            },
            "ARUCO_CONFIG": {
                "dictionary": "DICT_4X4_100",
                "marker_border_color": [0, 255, 0],
                "marker_border_thickness": 2,
                "text_color": [255, 255, 255],
                "text_thickness": 2,
                "text_font": "FONT_HERSHEY_SIMPLEX",
                "text_scale": 0.7
            },
            "JPEG_CONFIG": {
                "quality": 85,
                "optimize": True
            }
        }

