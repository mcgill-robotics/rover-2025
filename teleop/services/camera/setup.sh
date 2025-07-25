#!/bin/bash
# Setup script for multi-camera streaming system

echo "Setting up multi-camera streaming system..."

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install requirements
echo "Installing Python requirements..."
pip install --upgrade pip
pip install -r requirements.txt

# Make scripts executable
chmod +x jetson_server.py
chmod +x central_backend.py
chmod +x run_jetson.sh
chmod +x run_backend.sh

echo "Setup complete!"
echo ""
echo "Usage:"
echo "  1. On Jetson/Pi devices: ./run_jetson.sh --backend-host <BACKEND_IP>"
echo "  2. On central server: ./run_backend.sh"
echo "  3. Update frontend backend URL in MultiCameraView.tsx"
echo ""
echo "Example:"
echo "  # On Jetson (replace 192.168.1.100 with your backend server IP)"
echo "  ./run_jetson.sh --backend-host 192.168.1.100 --device-id jetson-01"
echo ""
echo "  # On central backend server"
echo "  ./run_backend.sh --http-port 8001 --udp-port 9999"
