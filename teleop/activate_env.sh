#!/bin/bash
# Teleop Environment Activation Script
# Source this script to activate the teleop Python environment

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

if [ -d "$VENV_DIR" ]; then
    echo -e "${GREEN}🐍 Activating teleop Python environment...${NC}"
    source "$VENV_DIR/bin/activate"
    echo -e "${GREEN}✅ Environment activated. Python path: $(which python)${NC}"
    
    # Check if ROS2 is available
    if command -v ros2 &> /dev/null; then
        if python -c "import rclpy" 2>/dev/null; then
            echo -e "${GREEN}🤖 ROS2 Python packages available${NC}"
        else
            echo -e "${YELLOW}⚠️  ROS2 detected but Python packages not in venv${NC}"
            echo -e "${BLUE}💡 For ROS features, also run: source /opt/ros/humble/setup.bash${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  ROS2 not found - some features may be limited${NC}"
    fi
    
    echo ""
    echo -e "${BLUE}📦 Key installed packages:${NC}"
    pip list --format=columns | grep -E "(fastapi|opencv|numpy|websockets|aiohttp)" || echo "  (Run 'pip list' to see all packages)"
    echo ""
    echo -e "${BLUE}💡 To deactivate: ${YELLOW}deactivate${NC}"
    echo -e "${BLUE}💡 For ROS features: ${YELLOW}source /opt/ros/humble/setup.bash${NC}"
else
    echo -e "${RED}❌ Virtual environment not found at $VENV_DIR${NC}"
    echo "Run ./setup_env.sh first to create the environment"
    return 1
fi
