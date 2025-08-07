#!/usr/bin/env python3
"""
Configuration loader for vision server.
Loads settings from config.yml file.
"""

import os
import yaml
from typing import Dict, Any

def get_jetson_config() -> Dict[str, Any]:
    """
    Load Jetson configuration from config.yml file.
    
    Returns:
        Dictionary containing configuration settings
    """
    config_path = os.path.join(os.path.dirname(__file__), 'config.yml')
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Return the jetson_config section
        return config.get('jetson_config', {})
        
    except (FileNotFoundError, yaml.YAMLError) as e:
        print(f"Warning: Could not load config.yml: {e}")
        # Return default config
        return {
            "default_backend_host": "192.168.1.100",
            "default_backend_port": 9999,
            "default_fps": 20,
            "capture_width": 640,
            "capture_height": 480,
            "gstreamer_config": {
                "h264_bitrate": 512,
                "h264_tune": "zerolatency"
            }
        }

def get_legacy_jetson_config() -> Dict[str, Any]:
    """
    Get configuration in legacy format for compatibility.
    
    Returns:
        Dictionary containing configuration in legacy format
    """
    config = get_jetson_config()
    
    # Convert to legacy format
    return {
        "DEFAULT_BACKEND_HOST": config.get("default_backend_host", "192.168.1.100"),
        "DEFAULT_BACKEND_PORT": config.get("default_backend_port", 9999),
        "DEFAULT_FPS": config.get("default_fps", 20),
        "CAPTURE_WIDTH": config.get("capture_width", 640),
        "CAPTURE_HEIGHT": config.get("capture_height", 480),
        "GSTREAMER_CONFIG": config.get("gstreamer_config", {
            "H264_BITRATE": config.get("gstreamer_config", {}).get("h264_bitrate", 512),
            "H264_TUNE": config.get("gstreamer_config", {}).get("h264_tune", "zerolatency")
        })
    } 