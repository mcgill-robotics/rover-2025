# Multi-Camera System Configuration

# Central Backend Server Configuration
BACKEND_CONFIG = {
    # Backend server host (where central_backend.py runs)
    "HOST": "0.0.0.0",  # Listen on all interfaces
    
    # HTTP port for REST API and WebSocket connections
    "HTTP_PORT": 8001,
    
    # UDP port for receiving frames from Jetson/Pi devices (legacy, now unused)
    "UDP_PORT": 9999,
    
    # Camera inactive timeout (seconds)
    "INACTIVE_TIMEOUT": 30.0,
    
    # Heartbeat timeout (seconds) - legacy, now unused
    "HEARTBEAT_TIMEOUT": 15.0,
    
    # Frame buffer size per camera
    "FRAME_BUFFER_SIZE": 10,
    
    # Maximum UDP packet size (legacy, now unused)
    "MAX_UDP_PACKET_SIZE": 65507,
    
    # Default camera configuration for GStreamer RTP streams
    "DEFAULT_CAMERAS": [
        {
            "camera_id": "jetson-01-cam00",
            "port": 5000,
            "device_id": "jetson-01",
            "name": "Front Camera"
        },
        {
            "camera_id": "jetson-01-cam01", 
            "port": 5001,
            "device_id": "jetson-01",
            "name": "Left Camera"
        },
        {
            "camera_id": "jetson-01-cam02",
            "port": 5002, 
            "device_id": "jetson-01",
            "name": "Right Camera"
        }
    ],
    
    # GStreamer pipeline settings
    "GSTREAMER_CONFIG": {
        "RTP_CAPS": "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
        "PIPELINE_ELEMENTS": [
            "rtph264depay",
            "h264parse", 
            "avdec_h264",
            "videoconvert",
            "video/x-raw,format=BGR",
            "appsink drop=true max-buffers=2"
        ],
        "BUFFER_SIZE": 1,  # Minimize latency
        "DROP_FRAMES": True
    },
    
    # ArUco detection settings
    "ARUCO_CONFIG": {
        "DICTIONARY": "DICT_4X4_100",
        "MARKER_BORDER_COLOR": [0, 255, 0],  # Green
        "MARKER_BORDER_THICKNESS": 2,
        "TEXT_COLOR": [255, 255, 255],  # White
        "TEXT_THICKNESS": 2,
        "TEXT_FONT": "FONT_HERSHEY_SIMPLEX",
        "TEXT_SCALE": 0.7
    },
    
    # JPEG encoding settings
    "JPEG_CONFIG": {
        "QUALITY": 85,
        "OPTIMIZE": True
    },

    # Camera names that should be inverted (upside down)
    "INVERTED_CAMERAS": [
        "Left Camera",  # This camera is mounted upside down
        # Add other camera names that need inversion here
    ]
}

# Jetson/Pi Device Configuration
JETSON_CONFIG = {
    # Default backend host for Jetson/Pi devices to send RTP streams to
    "DEFAULT_BACKEND_HOST": "192.168.1.100",
    
    # Default backend UDP port for Jetson/Pi devices (legacy heartbeat)
    "DEFAULT_BACKEND_PORT": 9999,
    
    # Default JPEG quality (1-100) - legacy, now unused
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
    
    # GStreamer settings for Jetson
    "GSTREAMER_CONFIG": {
        "H264_BITRATE": 512,  # kbps
        "H264_TUNE": "zerolatency",
        "RTP_PAYLOAD_TYPE": 96,
        "RTP_CONFIG_INTERVAL": 1,
        "UDP_BUFFER_SIZE": 1024 * 1024,  # 1MB
        "FRAME_BUFFER_SIZE": 65536,
        "MAX_FRAME_SIZE": 60000
    },
    
    # Camera port mapping (matches backend default cameras)
    "CAMERA_PORT_MAPPING": {
        "cam00": 5000,  # Front Camera
        "cam01": 5001,  # Left Camera  
        "cam02": 5002   # Right Camera
    }
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
