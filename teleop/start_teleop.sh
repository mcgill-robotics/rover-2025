#!/bin/bash

# Teleop System Startup Script
# This script starts all necessary services for the robot controller UI

set -e  # Exit on error

# ────── Source ROS Environment ──────
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Starting Mars Rover Teleop System..."

# ────── Terminal Colors ──────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ────── Utility Functions ──────
command_exists() { command -v "$1" >/dev/null 2>&1; }

port_in_use() { lsof -i :$1 >/dev/null 2>&1; }

web_port_up() {
  curl -s --head "http://localhost:$1" | head -n 1 | grep "HTTP/" >/dev/null
}

# ────── Prerequisite Check ──────
echo -e "${BLUE}Checking prerequisites...${NC}"

for cmd in ros2 node python3; do
  if ! command_exists $cmd; then
    echo -e "${RED}❌ $cmd not found. Please install or source it.${NC}"
    exit 1
  fi
done

echo -e "${GREEN}✅ Prerequisites check passed${NC}"

# ────── Directory Setup ──────
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SERVICES_DIR="$SCRIPT_DIR/services"
UI_DIR="$SCRIPT_DIR/robot-controller-ui"

[ -d "$SERVICES_DIR" ] || { echo -e "${RED}❌ Missing services dir: $SERVICES_DIR${NC}"; exit 1; }
[ -d "$UI_DIR" ] || { echo -e "${RED}❌ Missing UI dir: $UI_DIR${NC}"; exit 1; }

# ────── Dependency Install ──────
echo -e "${BLUE}Checking dependencies...${NC}"

if [ ! -f "$SERVICES_DIR/.deps_installed" ]; then
  echo -e "${YELLOW}Installing Python dependencies...${NC}"
  cd "$SERVICES_DIR"
  pip3 install -r requirements.txt
  touch .deps_installed
fi

if [ ! -d "$UI_DIR/node_modules" ]; then
  echo -e "${YELLOW}Installing Node.js dependencies...${NC}"
  cd "$UI_DIR"
  npm install
fi

echo -e "${GREEN}✅ Dependencies are ready${NC}"

# ────── Kill Processes on Used Ports ──────
if port_in_use $port; then
  echo -e "${YELLOW}⚠️ Port $port is in use — killing process...${NC}"
  lsof -ti:$port | xargs kill -9 || true
  sleep 1
fi

# ────── Exit Cleanup ──────
cleanup() {
  echo -e "\n${YELLOW}🛑 Shutting down services...${NC}"
  kill $ROS_MANAGER_PID $FRONTEND_PID 2>/dev/null || true
  lsof -ti:8082 | xargs kill -9 2>/dev/null || true
  lsof -ti:3000 | xargs kill -9 2>/dev/null || true
  echo -e "${GREEN}✅ Cleanup complete${NC}"
  exit 0
}
trap cleanup SIGINT SIGTERM

# ────── Start ROS Manager ──────
echo -e "${BLUE}🔧 Starting ROS Manager...${NC}"
cd "$SERVICES_DIR/ros"
python3 ros_manager.py &
ROS_MANAGER_PID=$!

echo -e "${YELLOW}Waiting for ROS Manager to start on port 8082...${NC}"
for i in {1..30}; do
  if web_port_up 8082; then
    echo -e "${GREEN}✅ ROS Manager is live on port 8082${NC}"
    break
  fi
  sleep 1
  [ $i -eq 30 ] && echo -e "${RED}❌ ROS Manager failed to start${NC}" && cleanup
done

# ────── Start Frontend ──────
echo -e "${BLUE}🌐 Starting Frontend Server...${NC}"
cd "$UI_DIR"
npm run dev 2>&1 | tee /tmp/frontend.log &
FRONTEND_PID=$!

echo -e "${YELLOW}Waiting for Frontend to start on port 3000...${NC}"
for i in {1..30}; do
  if web_port_up 3000; then
    echo -e "${GREEN}✅ Frontend is live on port 3000${NC}"
    break
  fi
  sleep 1
  [ $i -eq 30 ] && echo -e "${RED}❌ Frontend failed to start${NC}" && cleanup
done

# ────── Final Output ──────
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo -e "\n${GREEN}✅ Teleop System Started Successfully!${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN} Web Interface:${NC}     http://$LOCAL_IP:3000"
echo -e "${GREEN} ROS Manager API:${NC}   http://localhost:8082"
echo -e "${GREEN} WebSocket:${NC}         ws://localhost:8082/ws"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

echo -e "\n${YELLOW}Next Steps:${NC}"
echo -e "1. Ensure ROS2 environment is sourced"
echo -e "2. Start drive firmware node: ${BLUE}ros2 run control drive_firmware_node${NC}"
echo -e "3. Open the web UI: ${BLUE}http://localhost:3000${NC}"
echo -e "\n${YELLOW}To stop everything, press Ctrl+C in this terminal${NC}"

# ────── Keep Script Alive ──────
while true; do
  kill -0 $ROS_MANAGER_PID 2>/dev/null || { echo -e "${RED}❌ ROS Manager crashed${NC}"; cleanup; }
  kill -0 $FRONTEND_PID 2>/dev/null || { echo -e "${RED}❌ Frontend crashed${NC}"; cleanup; }
  sleep 5
done
