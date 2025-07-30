# Multi-Camera System Configuration

# Central Backend Server Configuration
BACKEND_CONFIG = {
    # Backend server host (where central_backend.py runs)
    "HOST": "192.168.1.100",  # Listen on all interfaces
    
    # HTTP port for REST API and WebSocket connections
    "HTTP_PORT": 8001,
    
    # UDP port for receiving frames from Jetson/Pi devices
    "UDP_PORT": 9999,
    
    # Camera inactive timeout (seconds)
    "INACTIVE_TIMEOUT": 30.0,
    
    # Heartbeat timeout (seconds)
    "HEARTBEAT_TIMEOUT": 15.0,
    
    # Frame buffer size per camera
    "FRAME_BUFFER_SIZE": 10,
    
    # Maximum UDP packet size
    "MAX_UDP_PACKET_SIZE": 65507,
}

# Jetson/Pi Device Configuration
JETSON_CONFIG = {
    # Default backend host for Jetson/Pi devices to send frames to
    "DEFAULT_BACKEND_HOST": "192.168.1.100",
    
    # Default backend UDP port for Jetson/Pi devices
    "DEFAULT_BACKEND_PORT": 9999,
    
    # Default JPEG quality (1-100)
    "DEFAULT_JPEG_QUALITY": 80,
    
    # Default target FPS
    "DEFAULT_FPS": 20,
    
    # Camera capture settings
    "CAPTURE_WIDTH": 640,
    "CAPTURE_HEIGHT": 480,
    
    # Heartbeat interval (seconds)
    "HEARTBEAT_INTERVAL": 5.0,
    
    # Frame send interval (calculated from FPS)
    "FRAME_INTERVAL": 1.0 / 20,  # 20 FPS default
}

# Network Configuration
NETWORK_CONFIG = {
    # Maximum retries for network operations
    "MAX_RETRIES": 3,
    
    # Connection timeout (seconds)
    "CONNECTION_TIMEOUT": 5.0,
    
    # Socket buffer sizes
    "UDP_SEND_BUFFER_SIZE": 1024 * 1024,  # 1MB
    "UDP_RECV_BUFFER_SIZE": 1024 * 1024,  # 1MB
}

# Logging Configuration
LOGGING_CONFIG = {
    "LEVEL": "INFO",
    "FORMAT": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    "DATE_FORMAT": "%Y-%m-%d %H:%M:%S",
}

# Helper functions
def get_backend_config():
    """Get backend configuration dictionary."""
    return BACKEND_CONFIG.copy()

def get_jetson_config():
    """Get Jetson/Pi configuration dictionary."""
    return JETSON_CONFIG.copy()

def get_network_config():
    """Get network configuration dictionary."""
    return NETWORK_CONFIG.copy()

def get_logging_config():
    """Get logging configuration dictionary."""
    return LOGGING_CONFIG.copy()

# Update frame interval based on FPS
def update_fps(fps):
    """Update frame interval based on target FPS."""
    JETSON_CONFIG["FRAME_INTERVAL"] = 1.0 / max(1, fps)
    return JETSON_CONFIG["FRAME_INTERVAL"]
