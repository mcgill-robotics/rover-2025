#!/bin/bash

# Teleop System Environment Activation Script

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Activating Teleop System Environment${NC}"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo -e "${RED}❌ Virtual environment not found. Please run setup_env.sh first.${NC}"
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)/services:$(pwd)/services/ros:$(pwd)/services/gps:$(pwd)/services/camera"

# Set ROS environment if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS environment detected: $ROS_DISTRO${NC}"
fi

echo -e "${GREEN}✅ Teleop system environment activated${NC}"
echo -e "${BLUE}🐍 Python: $(which python)${NC}"
echo -e "${BLUE}📦 Pip: $(which pip)${NC}"
echo -e "${BLUE}🔧 PYTHONPATH: $PYTHONPATH${NC}"
echo ""
echo -e "${GREEN}Ready to run teleop system!${NC}"
echo -e "${BLUE}Examples:${NC}"
echo -e "${BLUE}  Start system: ./start-teleop-system.sh${NC}"
echo -e "${BLUE}  Test services: cd services && ./test_service.sh camera${NC}"
echo -e "${BLUE}  Start UI: cd robot-controller-ui && npm run dev${NC}" 