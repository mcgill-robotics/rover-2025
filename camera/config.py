#!/usr/bin/env python3
"""
Configuration loader for vision system.
Loads settings from config.yml.
"""

import os
import yaml
import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)

def load_config() -> Dict[str, Any]:
    """Load configuration from config.yml."""
    config_path = os.path.join(os.path.dirname(__file__), 'config.yml')
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            logger.info(f"Loaded configuration from {config_path}")
            return config
    except (FileNotFoundError, yaml.YAMLError) as e:
        logger.error(f"Failed to load config file {config_path}: {e}")
        return {}

def get_jetson_config() -> Dict[str, Any]:
    """Get Jetson/Pi device configuration."""
    config = load_config()
    return config.get('jetson_config', {})

def get_network_config() -> Dict[str, Any]:
    """Get network configuration."""
    config = load_config()
    return config.get('network_config', {})

def get_logging_config() -> Dict[str, Any]:
    """Get logging configuration."""
    config = load_config()
    return config.get('logging_config', {})

def setup_logging():
    """Setup logging based on configuration."""
    config = get_logging_config()
    
    # Create formatter
    formatter = logging.Formatter(
        fmt=config.get('format', '%(asctime)s - %(name)s - %(levelname)s - %(message)s'),
        datefmt=config.get('date_format', '%Y-%m-%d %H:%M:%S')
    )
    
    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(config.get('level', 'INFO'))
    
    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Setup console logging
    if config.get('console_enabled', True):
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(logging.Formatter(config.get('console_format', '%(levelname)s: %(message)s')))
        root_logger.addHandler(console_handler)
    
    # Setup file logging
    if config.get('file_enabled', False):
        try:
            from logging.handlers import RotatingFileHandler
            
            file_path = config.get('file_path', '/var/log/vision_server.log')
            max_size = config.get('file_max_size', 10 * 1024 * 1024)  # 10MB
            backup_count = config.get('file_backup_count', 5)
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            file_handler = RotatingFileHandler(
                file_path,
                maxBytes=max_size,
                backupCount=backup_count
            )
            file_handler.setFormatter(formatter)
            root_logger.addHandler(file_handler)
            
            logger.info(f"File logging enabled: {file_path}")
        except Exception as e:
            logger.error(f"Failed to setup file logging: {e}")

def get_gstreamer_pipeline(camera_type: str, width: int, height: int, fps: int, bitrate: int, tune: str) -> str:
    """
    Build GStreamer pipeline string based on camera type and settings.
    
    Args:
        camera_type: One of 'mjpg', 'yuyv', or 'raw'
        width: Video width
        height: Video height
        fps: Target framerate
        bitrate: H.264 bitrate in kbps
        tune: H.264 encoder tune setting
        
    Returns:
        GStreamer pipeline string
    """
    config = get_jetson_config()
    gst_config = config.get('gstreamer_config', {})
    pipeline_formats = gst_config.get('pipeline_formats', {})
    
    if camera_type not in pipeline_formats:
        logger.warning(f"Unknown camera type: {camera_type}, falling back to raw")
        camera_type = 'raw'
    
    format_config = pipeline_formats[camera_type]
    
    # Build source caps
    if camera_type == 'raw':
        caps = format_config['caps'].format(
            width=width,
            height=height,
            fps=fps
        )
    else:
        caps = format_config['caps']
    
    # Build encoder elements
    encoder_elements = [
        element.format(tune=tune, bitrate=bitrate)
        for element in gst_config.get('encoder_elements', [])
    ]
    
    # Combine all elements
    elements = (
        ["v4l2src device={device_path}", "!", caps] +
        ["!"] + format_config['elements'] +
        ["!"] + encoder_elements +
        ["!", "udpsink host={host} port={port}"]
    )
    
    return " ".join(elements)

def validate_camera_settings(bitrate: int, fps: int) -> tuple[bool, str]:
    """
    Validate camera settings against configuration limits.
    
    Args:
        bitrate: Target bitrate in kbps
        fps: Target framerate
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    config = get_jetson_config()
    gst_config = config.get('gstreamer_config', {})
    
    min_bitrate = gst_config.get('min_bitrate', 128)
    max_bitrate = gst_config.get('max_bitrate', 2048)
    min_fps = gst_config.get('min_fps', 5)
    max_fps = gst_config.get('max_fps', 30)
    
    if not (min_bitrate <= bitrate <= max_bitrate):
        return False, f"Bitrate must be between {min_bitrate} and {max_bitrate} kbps"
    
    if not (min_fps <= fps <= max_fps):
        return False, f"FPS must be between {min_fps} and {max_fps}"
    
    return True, ""