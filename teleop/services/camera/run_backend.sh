#!/bin/bash
# Run script for GStreamer-based central backend server

# Load default values from config.py
eval $(python3 -c "
from config import get_backend_config
config = get_backend_config()
print(f'HTTP_PORT={config[\"HTTP_PORT\"]}')
print(f'INACTIVE_TIMEOUT={config[\"INACTIVE_TIMEOUT\"]}')
")

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --http-port)
            HTTP_PORT="$2"
            shift 2
            ;;
        --inactive-timeout)
            INACTIVE_TIMEOUT="$2"
            shift 2
            ;;
        --enable-aruco)
            ENABLE_ARUCO="--enable-aruco"
            shift
            ;;
        --disable-aruco)
            ENABLE_ARUCO="--disable-aruco"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "GStreamer-based Multi-Camera Backend Server"
            echo "Receives RTP/H.264 streams from Jetson devices via GStreamer pipelines"
            echo ""
            echo "Options:"
            echo "  --http-port PORT          HTTP port for API and WebSocket (default: $HTTP_PORT)"
            echo "  --inactive-timeout SEC    Timeout for inactive cameras in seconds (default: $INACTIVE_TIMEOUT)"
            echo "  --enable-aruco            Enable ArUco marker detection (default)"
            echo "  --disable-aruco           Disable ArUco marker detection"
            echo "  -h, --help                Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --http-port 8001 --enable-aruco"
            echo ""
            echo "API Endpoints:"
            echo "  GET  http://localhost:$HTTP_PORT/api/cameras         - List active cameras"
            echo "  POST http://localhost:$HTTP_PORT/api/cameras/add     - Add new camera"
            echo "  DEL  http://localhost:$HTTP_PORT/api/cameras/{id}    - Remove camera"
            echo "  GET  http://localhost:$HTTP_PORT/api/health          - Health check"
            echo "  WS   ws://localhost:$HTTP_PORT/stream?camera_id=X    - Stream camera X"
            echo ""
            echo "Requirements:"
            echo "  - GStreamer 1.0 with plugins (good, bad, ugly, libav)"
            echo "  - OpenCV with GStreamer support"
            echo "  - Jetson devices sending RTP/H.264 streams"
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
    echo "  Ubuntu/Debian: sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav"
    echo "  macOS: brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav"
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
python3 -c "import cv2, fastapi, uvicorn" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "ERROR: Missing Python dependencies!"
    echo "Please install required packages:"
    echo "  pip install fastapi uvicorn websockets opencv-python numpy"
    exit 1
fi

# Check OpenCV GStreamer support
python3 -c "import cv2; print('OpenCV GStreamer support:', cv2.getBuildInformation().find('GStreamer') != -1)" 2>/dev/null

echo "=========================================="
echo "GStreamer Multi-Camera Backend Server"
echo "=========================================="
echo "HTTP Port: $HTTP_PORT (API and WebSocket)"
echo "Inactive Timeout: ${INACTIVE_TIMEOUT}s"
echo "ArUco Detection: $([ -n "$ENABLE_ARUCO" ] && echo "Enabled" || echo "Enabled (default)")"
echo ""
echo "API Endpoints:"
echo "  GET  http://localhost:$HTTP_PORT/api/cameras"
echo "  POST http://localhost:$HTTP_PORT/api/cameras/add"
echo "  DEL  http://localhost:$HTTP_PORT/api/cameras/{id}"
echo "  GET  http://localhost:$HTTP_PORT/api/health"
echo "  WS   ws://localhost:$HTTP_PORT/stream?camera_id=X"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Run the server
python3 central_backend.py \
    --http-port "$HTTP_PORT" \
    --inactive-timeout "$INACTIVE_TIMEOUT" \
    $ENABLE_ARUCO
