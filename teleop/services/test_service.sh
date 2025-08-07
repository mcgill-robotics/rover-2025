#!/bin/bash

# Individual Service Tester
# Launches any service individually for development and testing

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to show usage
show_usage() {
    echo -e "${BLUE}Individual Service Tester${NC}"
    echo "=================================="
    echo ""
    echo "Usage: $0 [SERVICE]"
    echo ""
    echo "Available services:"
    echo "  camera    - Camera service (Port 8001)"
    echo "  gps       - GPS service (Port 5001)"
    echo "  ros       - ROS manager (Port 8082)"
    echo "  tileserver - TileServer-GL (Port 8080)"
    echo ""
    echo "Examples:"
    echo "  $0 camera     # Start camera service"
    echo "  $0 gps        # Start GPS service"
    echo "  $0 ros        # Start ROS manager"
    echo "  $0 tileserver # Start TileServer"
    echo ""
    echo "Note: Each service will be started in standalone mode for development and testing."
}

# Check if service name is provided
if [ $# -eq 0 ]; then
    show_usage
    exit 1
fi

SERVICE=$1

case $SERVICE in
    "camera")
        echo -e "${BLUE}📷 Launching Camera Service...${NC}"
        cd camera
        ./start_camera_service.sh
        ;;
    "gps")
        echo -e "${BLUE}📍 Launching GPS Service...${NC}"
        cd gps
        ./start_gps_service.sh
        ;;
    "ros")
        echo -e "${BLUE}🤖 Launching ROS Manager...${NC}"
        cd ros
        ./start_ros_manager.sh
        ;;
    "tileserver")
        echo -e "${BLUE}🗺️  Launching TileServer-GL...${NC}"
        cd gps
        if [ ! -f "docker-compose.tileserver.yml" ]; then
            echo -e "${RED}❌ docker-compose.tileserver.yml not found${NC}"
            exit 1
        fi
        echo -e "${YELLOW}🚀 Starting TileServer-GL...${NC}"
        echo -e "${BLUE}🌐 TileServer: http://localhost:8080${NC}"
        echo -e "${BLUE}📝 Logs: docker logs tileserver-gl${NC}"
        echo ""
        docker-compose -f docker-compose.tileserver.yml up
        ;;
    *)
        echo -e "${RED}❌ Unknown service: $SERVICE${NC}"
        echo ""
        show_usage
        exit 1
        ;;
esac 