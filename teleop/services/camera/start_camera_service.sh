#!/bin/bash

# Camera Service Standalone Launcher
# Launches only the camera service for individual testing

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}📷 Starting Camera Service (Standalone)${NC}"
echo "=================================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check Python
if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed${NC}"
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "camera_service.py" ]; then
    echo -e "${RED}❌ camera_service.py not found. Please run this script from the services/camera directory.${NC}"
    exit 1
fi

# Check dependencies
echo -e "${YELLOW}🔍 Checking dependencies...${NC}"

# Check if required Python packages are installed
python3 -c "import fastapi, uvicorn, cv2, numpy" 2>/dev/null || {
    echo -e "${RED}❌ Missing required Python packages. Please install:${NC}"
    echo "   pip install fastapi uvicorn opencv-contrib-python numpy"
    exit 1
}

# Check if service_config.yml exists
if [ ! -f "../service_config.yml" ]; then
    echo -e "${RED}❌ service_config.yml not found in parent directory${NC}"
    exit 1
fi

# Create logs directory
mkdir -p logs

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)/.."

echo -e "${GREEN}✅ Dependencies check passed${NC}"

# Display service information
echo -e "${BLUE}📊 Camera Service Information:${NC}"
echo -e "${BLUE}🌐 API: http://localhost:8001${NC}"
echo -e "${BLUE}📝 Logs: logs/camera_service.log${NC}"
echo -e "${BLUE}🔧 Config: ../service_config.yml${NC}"
echo ""

# Check if port is available
if lsof -Pi :8001 -sTCP:LISTEN -t >/dev/null ; then
    echo -e "${YELLOW}⚠️  Port 8001 is already in use${NC}"
    echo -e "${YELLOW}   You may need to stop the existing service first${NC}"
    echo ""
fi

echo -e "${YELLOW}🚀 Starting Camera Service...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the service${NC}"
echo ""

# Start the camera service
python3 camera_service.py 2>&1 | tee logs/camera_service.log 