#!/bin/bash

# Teleop System Startup Script
# This script starts all necessary services for the robot controller UI

set -e  # Exit on any error

echo "🚀 Starting Mars Rover Teleop System..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if a port is in use
port_in_use() {
    lsof -i :$1 >/dev/null 2>&1
}

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}"

if ! command_exists ros2; then
    echo -e "${RED}❌ ROS2 not found. Please source your ROS2 environment:${NC}"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/ros2_ws/install/setup.bash"
    exit 1
fi

if ! command_exists node; then
    echo -e "${RED}❌ Node.js not found. Please install Node.js 18+${NC}"
    exit 1
fi

if ! command_exists python3; then
    echo -e "${RED}❌ Python3 not found. Please install Python 3.8+${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Prerequisites check passed${NC}"

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SERVICES_DIR="$SCRIPT_DIR/services"
UI_DIR="$SCRIPT_DIR/robot-controller-ui"

# Check if directories exist
if [ ! -d "$SERVICES_DIR" ]; then
    echo -e "${RED}❌ Services directory not found: $SERVICES_DIR${NC}"
    exit 1
fi

if [ ! -d "$UI_DIR" ]; then
    echo -e "${RED}❌ UI directory not found: $UI_DIR${NC}"
    exit 1
fi

# Install dependencies if needed
echo -e "${BLUE}Checking dependencies...${NC}"

# Check Python dependencies
if [ ! -f "$SERVICES_DIR/.deps_installed" ]; then
    echo -e "${YELLOW}Installing Python dependencies...${NC}"
    cd "$SERVICES_DIR"
    pip3 install -r requirements.txt
    touch .deps_installed
    echo -e "${GREEN}✅ Python dependencies installed${NC}"
fi

# Check Node.js dependencies
if [ ! -d "$UI_DIR/node_modules" ]; then
    echo -e "${YELLOW}Installing Node.js dependencies...${NC}"
    cd "$UI_DIR"
    npm install
    echo -e "${GREEN}✅ Node.js dependencies installed${NC}"
fi

# Check for running processes on required ports
if port_in_use 8082; then
    echo -e "${YELLOW}⚠️  Port 8082 is already in use (ROS Manager)${NC}"
    read -p "Kill existing process? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        lsof -ti:8082 | xargs kill -9 2>/dev/null || true
        sleep 2
    fi
fi

if port_in_use 3000; then
    echo -e "${YELLOW}⚠️  Port 3000 is already in use (Frontend)${NC}"
    read -p "Kill existing process? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        lsof -ti:3000 | xargs kill -9 2>/dev/null || true
        sleep 2
    fi
fi

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}🛑 Shutting down services...${NC}"
    
    # Kill background processes
    if [ ! -z "$ROS_MANAGER_PID" ]; then
        kill $ROS_MANAGER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$FRONTEND_PID" ]; then
        kill $FRONTEND_PID 2>/dev/null || true
    fi
    
    # Kill any remaining processes on our ports
    lsof -ti:8082 | xargs kill -9 2>/dev/null || true
    lsof -ti:3000 | xargs kill -9 2>/dev/null || true
    
    echo -e "${GREEN}✅ Cleanup complete${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start ROS Manager
echo -e "${BLUE}🔧 Starting ROS Manager...${NC}"
cd "$SERVICES_DIR/ros"
python3 ros_manager.py &
ROS_MANAGER_PID=$!

# Wait for ROS Manager to start
echo -e "${YELLOW}Waiting for ROS Manager to start...${NC}"
for i in {1..30}; do
    if port_in_use 8082; then
        echo -e "${GREEN}✅ ROS Manager started on port 8082${NC}"
        break
    fi
    sleep 1
    if [ $i -eq 30 ]; then
        echo -e "${RED}❌ ROS Manager failed to start${NC}"
        cleanup
        exit 1
    fi
done

# Start Frontend
echo -e "${BLUE}🌐 Starting Frontend Development Server...${NC}"
cd "$UI_DIR"
npm run dev &
FRONTEND_PID=$!

# Wait for Frontend to start
echo -e "${YELLOW}Waiting for Frontend to start...${NC}"
for i in {1..30}; do
    if port_in_use 3000; then
        echo -e "${GREEN}✅ Frontend started on port 3000${NC}"
        break
    fi
    sleep 1
    if [ $i -eq 30 ]; then
        echo -e "${RED}❌ Frontend failed to start${NC}"
        cleanup
        exit 1
    fi
done

# Display status
echo -e "\n${GREEN}🎉 Teleop System Started Successfully!${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}📱 Web Interface:${NC}     http://localhost:3000"
echo -e "${GREEN}🔧 ROS Manager API:${NC}   http://localhost:8082"
echo -e "${GREEN}🔌 WebSocket:${NC}         ws://localhost:8082/ws"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

echo -e "\n${YELLOW}📋 Next Steps:${NC}"
echo -e "1. Ensure ROS2 environment is sourced"
echo -e "2. Start drive firmware node: ${BLUE}ros2 run control drive_firmware_node${NC}"
echo -e "3. Open web interface: ${BLUE}http://localhost:3000${NC}"
echo -e "\n${YELLOW}💡 Tip:${NC} Check the connection status indicator in the web interface"
echo -e "${YELLOW}🛑 To stop:${NC} Press Ctrl+C in this terminal"

# Keep script running and monitor processes
while true; do
    # Check if ROS Manager is still running
    if ! kill -0 $ROS_MANAGER_PID 2>/dev/null; then
        echo -e "${RED}❌ ROS Manager stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    # Check if Frontend is still running
    if ! kill -0 $FRONTEND_PID 2>/dev/null; then
        echo -e "${RED}❌ Frontend stopped unexpectedly${NC}"
        cleanup
        exit 1
    fi
    
    sleep 5
done
