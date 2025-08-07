#!/bin/bash
# Run script for GStreamer-based Vision Server (Jetson/Pi)

# Load default values from config.py
eval $(python3 -c "
from config import get_jetson_config
config = get_jetson_config()
gst_config = config['GSTREAMER_CONFIG']
print(f'BACKEND_HOST=\"{config[\"DEFAULT_BACKEND_HOST\"]}\"')
print(f'BACKEND_PORT={config[\"DEFAULT_BACKEND_PORT\"]}')
print(f'FPS={config[\"DEFAULT_FPS\"]}')
print(f'WIDTH={config[\"CAPTURE_WIDTH\"]}')
print(f'HEIGHT={config[\"CAPTURE_HEIGHT\"]}')
print(f'BITRATE={gst_config[\"H264_BITRATE\"]}')
print(f'TUNE=\"{gst_config[\"H264_TUNE\"]}\"')
")

# Default device ID (not in config since it should be unique per device)
DEVICE_ID="jetson-01"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --backend-host)
            BACKEND_HOST="$2"
            shift 2
            ;;
        --backend-port)
            BACKEND_PORT="$2"
            shift 2
            ;;
        --device-id)
            DEVICE_ID="$2"
            shift 2
            ;;
        --width)
            WIDTH="$2"
            shift 2
            ;;
        --height)
            HEIGHT="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --bitrate)
            BITRATE="$2"
            shift 2
            ;;
        --tune)
            TUNE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "GStreamer-based Multi-Camera Vision Server"
            echo "Captures video from cameras and sends RTP/H.264 streams to backend"
            echo ""
            echo "Options:"
            echo "  --backend-host HOST    Backend server IP address (default: $BACKEND_HOST)"
            echo "  --backend-port PORT    Backend server UDP port for heartbeat (default: $BACKEND_PORT)"
            echo "  --device-id ID         Unique device identifier (default: $DEVICE_ID)"
            echo "  --width WIDTH          Video capture width (default: $WIDTH)"
            echo "  --height HEIGHT        Video capture height (default: $HEIGHT)"
            echo "  --fps FPS              Target frames per second (default: $FPS)"
            echo "  --bitrate KBPS         H.264 bitrate in kbps (default: $BITRATE)"
            echo "  --tune PRESET          H.264 encoder tune preset (default: $TUNE)"
            echo "  -h, --help             Show this help message"
            echo ""
            echo "Camera Port Mapping (RTP streams sent to backend):"
            echo "  cam00 -> Backend Port 5000 (Front Camera)"
            echo "  cam01 -> Backend Port 5001 (Left Camera)"
            echo "  cam02 -> Backend Port 5002 (Right Camera)"
            echo ""
            echo "Example:"
            echo "  $0 --backend-host 192.168.1.100 --device-id jetson-01 --fps 30"
            echo ""
            echo "Requirements:"
            echo "  - GStreamer 1.0 with NVIDIA plugins (nvvidconv, x264enc)"
            echo "  - v4l2-utils for camera detection"
            echo "  - Cameras connected to /dev/video* devices"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check for GStreamer installation
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo "ERROR: GStreamer not found!"
    echo "Please install GStreamer 1.0:"
    echo "  sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav"
    echo "  # For Jetson, also install NVIDIA plugins:"
    echo "  sudo apt-get install gstreamer1.0-plugins-nvvideo4linux2"
    exit 1
fi

# Check for v4l2-ctl
if ! command -v v4l2-ctl &> /dev/null; then
    echo "ERROR: v4l2-ctl not found!"
    echo "Please install v4l-utils:"
    echo "  sudo apt-get install v4l-utils"
    exit 1
fi

# Check if virtual environment exists
if [ -d "camera_venv" ]; then
    echo "Activating virtual environment..."
    source camera_venv/bin/activate
elif [ -d "venv" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
else
    echo "WARNING: No virtual environment found. Using system Python."
fi

# Check Python dependencies
python3 -c "import subprocess, json, socket, struct, time, threading" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ERROR: Missing Python dependencies!"
    echo "Standard library modules should be available."
    exit 1
fi

# Discover available cameras
echo "Discovering cameras..."
v4l2-ctl --list-devices 2>/dev/null | grep -E "(video|Camera)" || echo "No cameras found or v4l2-ctl failed"

echo "=============================================="
echo "GStreamer Multi-Camera Vision Server"
echo "=============================================="
echo "Backend: $BACKEND_HOST:$BACKEND_PORT (heartbeat)"
echo "Device ID: $DEVICE_ID"
echo "Video Settings: ${WIDTH}x${HEIGHT}@${FPS}fps"
echo "H.264 Settings: ${BITRATE}kbps, tune=$TUNE"
echo ""
echo "RTP Stream Ports (sent to backend):"
echo "  cam00 -> $BACKEND_HOST:5000 (Front Camera)"
echo "  cam01 -> $BACKEND_HOST:5001 (Left Camera)"
echo "  cam02 -> $BACKEND_HOST:5002 (Right Camera)"
echo ""
echo "The server will:"
echo "  1. Discover available cameras using v4l2-ctl"
echo "  2. Start GStreamer pipelines for each camera"
echo "  3. Send RTP/H.264 streams to backend ports 5000-5002"
echo "  4. Send periodic heartbeats to backend:$BACKEND_PORT"
echo ""
echo "Press Ctrl+C to stop"
echo "=============================================="

# Run the server
python3 vision_server.py \
    --backend-host "$BACKEND_HOST" \
    --backend-port "$BACKEND_PORT" \
    --device-id "$DEVICE_ID" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --bitrate "$BITRATE" \
    --tune "$TUNE"
