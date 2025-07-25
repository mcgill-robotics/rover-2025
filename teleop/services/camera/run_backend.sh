#!/bin/bash
# Run script for central backend server

# Load default values from config.py
eval $(python3 -c "
from config import get_backend_config
config = get_backend_config()
print(f'UDP_PORT={config[\"UDP_PORT\"]}')
print(f'HTTP_PORT={config[\"HTTP_PORT\"]}')
print(f'INACTIVE_TIMEOUT={config[\"INACTIVE_TIMEOUT\"]}')
print(f'HEARTBEAT_TIMEOUT={config[\"HEARTBEAT_TIMEOUT\"]}')
")

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --udp-port)
            UDP_PORT="$2"
            shift 2
            ;;
        --http-port)
            HTTP_PORT="$2"
            shift 2
            ;;
        --inactive-timeout)
            INACTIVE_TIMEOUT="$2"
            shift 2
            ;;
        --heartbeat-timeout)
            HEARTBEAT_TIMEOUT="$2"
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
            echo "Options:"
            echo "  --udp-port PORT           UDP port for receiving frames (default: $UDP_PORT)"
            echo "  --http-port PORT          HTTP port for API and WebSocket (default: $HTTP_PORT)"
            echo "  --inactive-timeout SEC    Timeout for inactive cameras in seconds (default: $INACTIVE_TIMEOUT)"
            echo "  --heartbeat-timeout SEC   Timeout for heartbeat in seconds (default: $HEARTBEAT_TIMEOUT)"
            echo "  --enable-aruco            Enable ArUco marker detection (default)"
            echo "  --disable-aruco           Disable ArUco marker detection"
            echo "  -h, --help                Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --http-port 8001 --udp-port 9999"
            echo ""
            echo "API Endpoints:"
            echo "  GET  http://localhost:$HTTP_PORT/api/cameras    - List active cameras"
            echo "  GET  http://localhost:$HTTP_PORT/api/health     - Health check"
            echo "  WS   ws://localhost:$HTTP_PORT/stream?camera_id=X - Stream camera X"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Please run setup.sh first."
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

echo "Starting central backend server..."
echo "UDP Port: $UDP_PORT (for receiving frames from Jetson/Pi devices)"
echo "HTTP Port: $HTTP_PORT (for API and WebSocket connections)"
echo "Inactive Timeout: ${INACTIVE_TIMEOUT}s"
echo "Heartbeat Timeout: ${HEARTBEAT_TIMEOUT}s"
echo ""
echo "API Endpoints:"
echo "  GET  http://localhost:$HTTP_PORT/api/cameras"
echo "  GET  http://localhost:$HTTP_PORT/api/health"
echo "  WS   ws://localhost:$HTTP_PORT/stream?camera_id=X"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the server
python3 central_backend.py \
    --udp-port "$UDP_PORT" \
    --http-port "$HTTP_PORT" \
    --inactive-timeout "$INACTIVE_TIMEOUT" \
    --heartbeat-timeout "$HEARTBEAT_TIMEOUT" \
    $ENABLE_ARUCO
