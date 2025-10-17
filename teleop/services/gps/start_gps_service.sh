#!/bin/bash

# GPS Service Standalone Launcher
# Launches only the GPS service for individual testing

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}📍 Starting GPS Service (Standalone)${NC}"
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
if [ ! -f "gps_service.py" ]; then
    echo -e "${RED}❌ gps_service.py not found. Please run this script from the services/gps directory.${NC}"
    exit 1
fi

# Check dependencies
echo -e "${YELLOW}🔍 Checking dependencies...${NC}"

# Check if required Python packages are installed
python3 -c "import flask, flask_cors, rclpy" 2>/dev/null || {
    echo -e "${RED}❌ Missing required Python packages. Please install:${NC}"
    echo "   pip install flask flask-cors rclpy"
    exit 1
}

# Check if service_config.yml exists
if [ ! -f "../service_config.yml" ]; then
    echo -e "${RED}❌ service_config.yml not found in parent directory${NC}"
    exit 1
fi

# Check ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS environment not sourced. GPS data may not be available.${NC}"
    echo -e "${YELLOW}   To enable ROS GPS data, run: source /opt/ros/humble/setup.bash${NC}"
    echo ""
fi

# Create logs directory
mkdir -p logs

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)/.."

echo -e "${GREEN}✅ Dependencies check passed${NC}"

# Display service information
echo -e "${BLUE}📊 GPS Service Information:${NC}"
echo -e "${BLUE}🌐 API: http://localhost:5001${NC}"
echo -e "${BLUE}📡 SSE Stream: http://localhost:5001/api/gps/stream${NC}"
echo -e "${BLUE}📝 Logs: logs/gps_service.log${NC}"
echo -e "${BLUE}🔧 Config: ../service_config.yml${NC}"
echo ""

# Check if port is available
if lsof -Pi :5001 -sTCP:LISTEN -t >/dev/null ; then
    echo -e "${YELLOW}⚠️  Port 5001 is already in use${NC}"
    echo -e "${YELLOW}   You may need to stop the existing service first${NC}"
    echo ""
fi

echo -e "${YELLOW}🚀 Starting GPS Service...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the service${NC}"
echo ""

# Start the GPS service
python3 gps_api.py 2>&1 | tee logs/gps_service.log 