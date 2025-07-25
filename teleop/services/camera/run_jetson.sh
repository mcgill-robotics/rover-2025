#!/bin/bash
# Run script for Jetson/Pi camera server

# Load default values from config.py
eval $(python3 -c "
from config import get_jetson_config
config = get_jetson_config()
print(f'BACKEND_HOST=\"{config[\"DEFAULT_BACKEND_HOST\"]}\"')
print(f'BACKEND_PORT={config[\"DEFAULT_BACKEND_PORT\"]}')
print(f'JPEG_QUALITY={config[\"DEFAULT_JPEG_QUALITY\"]}')
print(f'FPS={config[\"DEFAULT_FPS\"]}')
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
        --jpeg-quality)
            JPEG_QUALITY="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --backend-host HOST    Backend server IP address (default: $BACKEND_HOST)"
            echo "  --backend-port PORT    Backend server UDP port (default: $BACKEND_PORT)"
            echo "  --device-id ID         Unique device identifier (default: $DEVICE_ID)"
            echo "  --jpeg-quality QUAL    JPEG compression quality 1-100 (default: $JPEG_QUALITY)"
            echo "  --fps FPS              Target frames per second (default: $FPS)"
            echo "  -h, --help             Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --backend-host 192.168.1.100 --device-id jetson-01"
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

echo "Starting Jetson camera server..."
echo "Backend: $BACKEND_HOST:$BACKEND_PORT"
echo "Device ID: $DEVICE_ID"
echo "JPEG Quality: $JPEG_QUALITY"
echo "Target FPS: $FPS"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the server
python3 jetson_server.py \
    --backend-host "$BACKEND_HOST" \
    --backend-port "$BACKEND_PORT" \
    --device-id "$DEVICE_ID" \
    --jpeg-quality "$JPEG_QUALITY" \
    --fps "$FPS"
