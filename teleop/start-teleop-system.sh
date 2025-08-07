#!/bin/bash

# Rover 2025 Teleop System Startup Script
# This script starts the complete teleop system including:
# - GPS services and offline mapping
# - ROS integration and motor control
# - Camera services
# - React UI frontend
# - TileServer for offline maps
#
# Usage: ./start-teleop-system.sh
# Stop: Press Ctrl+C for graceful shutdown

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting Rover 2025 Teleop System${NC}"
echo "============================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if a port is in use
port_in_use() {
    lsof -i :$1 >/dev/null 2>&1
}

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command_exists docker; then
    echo -e "${RED}❌ Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

if ! command_exists node; then
    echo -e "${RED}❌ Node.js is not installed. Please install Node.js first.${NC}"
    exit 1
fi

if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed. Please install Python 3 first.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Prerequisites check passed${NC}"

# Check if map tiles exist
if [ ! -d "services/gps/mbtiles" ] || [ ! -f "services/gps/mbtiles/v3.mbtiles" ]; then
    echo -e "${YELLOW}⚠️  Map tiles not found. Downloading tiles...${NC}"
    cd services/gps
    ./download-map-tiles.sh
    cd ../..
fi

# Start TileServer-GL
echo -e "${YELLOW}Starting TileServer-GL...${NC}"
if port_in_use 8080; then
    echo -e "${YELLOW}⚠️  Port 8080 is already in use. Stopping existing service...${NC}"
    docker-compose -f services/gps/docker-compose.tileserver.yml down
fi

docker-compose -f services/gps/docker-compose.tileserver.yml up -d
echo -e "${GREEN}✅ TileServer-GL started at http://localhost:8080${NC}"

# Wait for TileServer to be ready
echo -e "${YELLOW}Waiting for TileServer to be ready...${NC}"
sleep 5

# Check if TileServer is responding
if curl -s http://localhost:8080 > /dev/null; then
    echo -e "${GREEN}✅ TileServer is responding${NC}"
else
    echo -e "${RED}❌ TileServer is not responding. Check logs with: docker-compose -f docker-compose.tileserver.yml logs${NC}"
fi

# Start Unified Service Manager
echo -e "${YELLOW}Starting Unified Service Manager...${NC}"
if [ -d "services" ]; then
    cd services
    
    # Check if required Python packages are installed
    if ! python3 -c "import aiohttp, flask, rclpy" 2>/dev/null; then
        echo -e "${YELLOW}Installing service dependencies...${NC}"
        pip3 install aiohttp aiohttp_cors flask flask_cors rclpy sensor_msgs geometry_msgs
    fi
    
    # Start unified service manager in background
    if ! port_in_use 8083; then
        python3 service_manager.py &
        SERVICE_MANAGER_PID=$!
        echo -e "${GREEN}✅ Unified Service Manager started (PID: $SERVICE_MANAGER_PID)${NC}"
        echo -e "${BLUE}📊 Service Manager API: http://localhost:8083${NC}"
        echo -e "${BLUE}🤖 ROS Manager API: http://localhost:8082${NC}"
        echo -e "${BLUE}📍 GPS Service API: http://localhost:5001${NC}"
    else
        echo -e "${YELLOW}⚠️  Service Manager already running on port 8083${NC}"
    fi
    
    cd ..
else
    echo -e "${YELLOW}⚠️  Services directory not found. Skipping service manager.${NC}"
fi

# Start React UI
echo -e "${YELLOW}Starting React UI...${NC}"
cd robot-controller-ui

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}Installing Node.js dependencies...${NC}"
    npm install --legacy-peer-deps
fi

# Start the development server
if ! port_in_use 3000; then
    echo -e "${GREEN}✅ Starting React development server...${NC}"
    echo -e "${BLUE}🌐 UI will be available at: http://localhost:3000${NC}"
    echo -e "${BLUE}🗺️  Mapping page: http://localhost:3000/mapping${NC}"
    npm run dev
else
    echo -e "${YELLOW}⚠️  Port 3000 is already in use. React UI may already be running.${NC}"
    echo -e "${BLUE}🌐 UI should be available at: http://localhost:3000${NC}"
fi

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Shutting down services...${NC}"
    
    # Stop Service Manager if running
    if [ ! -z "$SERVICE_MANAGER_PID" ]; then
        kill $SERVICE_MANAGER_PID 2>/dev/null || true
        echo -e "${GREEN}✅ Unified Service Manager stopped${NC}"
    fi
    
    # Stop TileServer
    docker-compose -f services/gps/docker-compose.tileserver.yml down
    echo -e "${GREEN}✅ TileServer-GL stopped${NC}"
    
    echo -e "${GREEN}✅ All services stopped${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Keep script running
echo -e "${GREEN}🎉 Teleop system is running!${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all services${NC}"
wait 