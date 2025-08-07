#!/bin/bash

# Teleop System Launcher for Rover 2025
# Starts the complete teleop system with configurable service modes

set -e

# Default mode
MODE="all"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--mode MODE]"
            echo ""
            echo "Modes:"
            echo "  all      - Start all services (default)"
            echo "  basic    - Start only ROS and GPS services"
            echo "  mapping  - Start ROS, GPS, and TileServer"
            echo "  camera   - Start ROS, GPS, and Camera service"
            echo "  ros-only - Start only ROS Manager"
            echo ""
            echo "Examples:"
            echo "  $0                    # Start all services"
            echo "  $0 --mode basic       # Start basic services only"
            echo "  $0 --mode camera      # Start with camera service"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting Teleop System for Rover 2025 (Mode: $MODE)${NC}"
echo "=================================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if a port is in use
port_in_use() {
    lsof -i :$1 >/dev/null 2>&1
}

# Function to check if ROS environment is available
check_ros_environment() {
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}⚠️  ROS environment not detected. Attempting to source ROS...${NC}"
        
        # Try to source ROS
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            echo -e "${GREEN}✅ Sourced ROS Humble${NC}"
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source /opt/ros/foxy/setup.bash
            echo -e "${GREEN}✅ Sourced ROS Foxy${NC}"
        elif [ -f "/opt/ros/noetic/setup.bash" ]; then
            source /opt/ros/noetic/setup.bash
            echo -e "${GREEN}✅ Sourced ROS Noetic${NC}"
        else
            echo -e "${RED}❌ ROS not found. Some services may not work properly.${NC}"
            echo -e "${YELLOW}Please install ROS or source your ROS environment manually.${NC}"
        fi
    else
        echo -e "${GREEN}✅ ROS environment detected: $ROS_DISTRO${NC}"
    fi
}

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed. Please install Python 3 first.${NC}"
    exit 1
fi

if ! command_exists pip3; then
    echo -e "${RED}❌ pip3 is not installed. Please install pip3 first.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Prerequisites check passed${NC}"

# Check ROS environment
check_ros_environment

# Check if required Python packages are installed
echo -e "${YELLOW}Checking Python dependencies...${NC}"

REQUIRED_PACKAGES=(
    "aiohttp"
    "aiohttp_cors"
    "flask"
    "flask_cors"
    "rclpy"
    "sensor_msgs"
    "geometry_msgs"
)

MISSING_PACKAGES=()

for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! python3 -c "import $package" 2>/dev/null; then
        MISSING_PACKAGES+=("$package")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    echo -e "${YELLOW}Installing missing packages: ${MISSING_PACKAGES[*]}${NC}"
    pip3 install "${MISSING_PACKAGES[@]}"
    echo -e "${GREEN}✅ Dependencies installed${NC}"
else
    echo -e "${GREEN}✅ All dependencies are installed${NC}"
fi

# Check if ports are available
echo -e "${YELLOW}Checking port availability...${NC}"

PORTS_TO_CHECK=(8082 5001 8083)
UNAVAILABLE_PORTS=()

for port in "${PORTS_TO_CHECK[@]}"; do
    if port_in_use $port; then
        UNAVAILABLE_PORTS+=("$port")
        echo -e "${YELLOW}⚠️  Port $port is already in use${NC}"
    fi
done

if [ ${#UNAVAILABLE_PORTS[@]} -ne 0 ]; then
    echo -e "${YELLOW}Some ports are in use. The service manager will attempt to use them anyway.${NC}"
    echo -e "${YELLOW}If you encounter issues, please stop the services using these ports: ${UNAVAILABLE_PORTS[*]}${NC}"
fi

# Create logs directory
mkdir -p logs

# Start the unified service manager
echo -e "${YELLOW}Starting Unified Service Manager...${NC}"

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export ROS_LOG_LEVEL=INFO

# Create a temporary config based on mode
TEMP_CONFIG="service_config_${MODE}.yml"
cp service_config.yml "$TEMP_CONFIG"

# Modify config based on mode
case $MODE in
    "basic")
        echo -e "${YELLOW}📡 Starting basic services (ROS + GPS)...${NC}"
        python3 -c "
import yaml
with open('$TEMP_CONFIG', 'r') as f:
    config = yaml.safe_load(f)
config['services']['tileserver']['enabled'] = False
config['services']['camera_service']['enabled'] = False
with open('$TEMP_CONFIG', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)
"
        ;;
    "mapping")
        echo -e "${YELLOW}🗺️  Starting mapping services (ROS + GPS + TileServer)...${NC}"
        python3 -c "
import yaml
with open('$TEMP_CONFIG', 'r') as f:
    config = yaml.safe_load(f)
config['services']['camera_service']['enabled'] = False
with open('$TEMP_CONFIG', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)
"
        ;;
    "camera")
        echo -e "${YELLOW}📷 Starting camera services (ROS + GPS + Camera)...${NC}"
        python3 -c "
import yaml
with open('$TEMP_CONFIG', 'r') as f:
    config = yaml.safe_load(f)
config['services']['tileserver']['enabled'] = False
with open('$TEMP_CONFIG', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)
"
        ;;
    "ros-only")
        echo -e "${YELLOW}🤖 Starting ROS Manager only...${NC}"
        python3 -c "
import yaml
with open('$TEMP_CONFIG', 'r') as f:
    config = yaml.safe_load(f)
config['services']['gps_service']['enabled'] = False
config['services']['tileserver']['enabled'] = False
config['services']['camera_service']['enabled'] = False
with open('$TEMP_CONFIG', 'w') as f:
    yaml.dump(config, f, default_flow_style=False, indent=2)
"
        ;;
    "all")
        echo -e "${YELLOW}📡 Starting all services...${NC}"
        ;;
    *)
        echo -e "${RED}❌ Unknown mode: $MODE${NC}"
        echo "Use --help for available modes"
        exit 1
        ;;
esac

# Start the service manager
echo -e "${GREEN}✅ Starting services...${NC}"
echo -e "${BLUE}📊 Service Manager API: http://localhost:8083${NC}"
echo -e "${BLUE}🤖 ROS Manager API: http://localhost:8082${NC}"
echo -e "${BLUE}📍 GPS Service API: http://localhost:5001${NC}"
if [[ "$MODE" == "camera" || "$MODE" == "all" ]]; then
    echo -e "${BLUE}📷 Camera Service API: http://localhost:8001${NC}"
fi
if [[ "$MODE" == "mapping" || "$MODE" == "all" ]]; then
    echo -e "${BLUE}🗺️  TileServer API: http://localhost:8080${NC}"
fi
echo -e "${BLUE}📝 Logs: logs/service_manager.log${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all services${NC}"

# Run the service manager with the modified config
python3 service_manager.py --config "$TEMP_CONFIG" 2>&1 | tee logs/service_manager.log

# Clean up temporary config
rm -f "$TEMP_CONFIG" 