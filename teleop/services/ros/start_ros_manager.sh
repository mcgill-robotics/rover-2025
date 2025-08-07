#!/bin/bash

# ROS Manager Standalone Launcher
# Launches only the ROS manager for individual testing

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🤖 Starting ROS Manager (Standalone)${NC}"
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
if [ ! -f "ros_manager.py" ]; then
    echo -e "${RED}❌ ros_manager.py not found. Please run this script from the services/ros directory.${NC}"
    exit 1
fi

# Check dependencies
echo -e "${YELLOW}🔍 Checking dependencies...${NC}"

# Check if required Python packages are installed
python3 -c "import rclpy, aiohttp, websockets" 2>/dev/null || {
    echo -e "${RED}❌ Missing required Python packages. Please install:${NC}"
    echo "   pip install rclpy aiohttp websockets"
    exit 1
}

# Check if service_config.yml exists
if [ ! -f "../service_config.yml" ]; then
    echo -e "${RED}❌ service_config.yml not found in parent directory${NC}"
    exit 1
fi

# Check ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS environment not sourced.${NC}"
    echo -e "${YELLOW}   To enable ROS functionality, run: source /opt/ros/humble/setup.bash${NC}"
    echo ""
fi

# Check if ROS is running
if ! command_exists ros2; then
    echo -e "${YELLOW}⚠️  ROS2 not found. Some functionality may be limited.${NC}"
    echo ""
fi

# Create logs directory
mkdir -p logs

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)/.."

echo -e "${GREEN}✅ Dependencies check passed${NC}"

# Display service information
echo -e "${BLUE}📊 ROS Manager Information:${NC}"
echo -e "${BLUE}🌐 API: http://localhost:8082${NC}"
echo -e "${BLUE}🔌 WebSocket: ws://localhost:8082/ws${NC}"
echo -e "${BLUE}📝 Logs: logs/ros_manager.log${NC}"
echo -e "${BLUE}🔧 Config: ../service_config.yml${NC}"
echo ""

# Check if port is available
if lsof -Pi :8082 -sTCP:LISTEN -t >/dev/null ; then
    echo -e "${YELLOW}⚠️  Port 8082 is already in use${NC}"
    echo -e "${YELLOW}   You may need to stop the existing service first${NC}"
    echo ""
fi

echo -e "${YELLOW}🚀 Starting ROS Manager...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the service${NC}"
echo ""

# Start the ROS manager
python3 ros_manager.py 2>&1 | tee logs/ros_manager.log 