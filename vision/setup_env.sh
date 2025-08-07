#!/bin/bash

# Vision Server Setup Script
# Lightweight setup for Jetson/Pi vision server

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Setting up Vision Server Environment${NC}"
echo "=============================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to detect system type
detect_system() {
    if [ -f "/etc/nv_tegra_release" ]; then
        echo "jetson"
    elif [ -f "/etc/rpi-issue" ] || [ -f "/proc/device-tree/model" ] && grep -q "Raspberry Pi" /proc/device-tree/model; then
        echo "raspberry_pi"
    else
        echo "linux"
    fi
}

SYSTEM_TYPE=$(detect_system)
echo -e "${BLUE}📱 Detected system: $SYSTEM_TYPE${NC}"

# Check prerequisites
echo -e "${YELLOW}🔍 Checking prerequisites...${NC}"

if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed${NC}"
    exit 1
fi

if ! command_exists pip3; then
    echo -e "${RED}❌ pip3 is not installed${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Basic prerequisites check passed${NC}"

# Install system dependencies
echo -e "${YELLOW}📦 Installing system dependencies...${NC}"

case $SYSTEM_TYPE in
    "jetson")
        echo -e "${BLUE}🤖 Installing Jetson dependencies...${NC}"
        sudo apt update
        sudo apt install -y \
            python3-dev \
            python3-pip \
            python3-venv \
            gstreamer1.0-tools \
            gstreamer1.0-plugins-base \
            gstreamer1.0-plugins-good \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-plugins-ugly \
            gstreamer1.0-libav \
            gstreamer1.0-plugins-nvenc \
            gstreamer1.0-plugins-nvdec \
            gstreamer1.0-plugins-nvvidconv \
            v4l-utils \
            libv4l-dev \
            libopencv-dev \
            libopencv-contrib-dev \
            python3-opencv \
            python3-gi \
            libgirepository1.0-dev
        ;;
        
    "raspberry_pi")
        echo -e "${BLUE}🍓 Installing Raspberry Pi dependencies...${NC}"
        sudo apt update
        sudo apt install -y \
            python3-dev \
            python3-pip \
            python3-venv \
            gstreamer1.0-tools \
            gstreamer1.0-plugins-base \
            gstreamer1.0-plugins-good \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-plugins-ugly \
            gstreamer1.0-libav \
            v4l-utils \
            libv4l-dev \
            libopencv-dev \
            libopencv-contrib-dev \
            python3-opencv \
            python3-gi \
            libgirepository1.0-dev
        ;;
        
    "linux")
        echo -e "${BLUE}🐧 Installing Linux dependencies...${NC}"
        if command_exists apt; then
            sudo apt update
            sudo apt install -y \
                python3-dev \
                python3-pip \
                python3-venv \
                gstreamer1.0-tools \
                gstreamer1.0-plugins-base \
                gstreamer1.0-plugins-good \
                gstreamer1.0-plugins-bad \
                gstreamer1.0-plugins-ugly \
                gstreamer1.0-libav \
                v4l-utils \
                libv4l-dev \
                libopencv-dev \
                libopencv-contrib-dev \
                python3-opencv \
                python3-gi \
                libgirepository1.0-dev
        else
            echo -e "${RED}❌ Unsupported package manager${NC}"
            exit 1
        fi
        ;;
esac

echo -e "${GREEN}✅ System dependencies installed${NC}"

# Create virtual environment
echo -e "${YELLOW}🐍 Creating Python virtual environment...${NC}"

if [ -d "venv" ]; then
    echo -e "${YELLOW}⚠️  Virtual environment already exists. Removing...${NC}"
    rm -rf venv
fi

python3 -m venv venv
echo -e "${GREEN}✅ Virtual environment created${NC}"

# Activate virtual environment
echo -e "${YELLOW}🔧 Activating virtual environment...${NC}"
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install only essential Python dependencies
echo -e "${YELLOW}📦 Installing Python dependencies...${NC}"

pip install \
    numpy>=1.23.0 \
    PyGObject>=3.42.0 \
    PyYAML>=6.0

echo -e "${GREEN}✅ Python dependencies installed${NC}"

# Create activation script
echo -e "${YELLOW}📝 Creating activation script...${NC}"

cat > activate_env.sh << 'EOF'
#!/bin/bash

# Vision Server Environment Activation Script

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Activating Vision Server Environment${NC}"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo -e "${RED}❌ Virtual environment not found. Please run setup_env.sh first.${NC}"
    exit 1
fi

# Activate virtual environment
source venv/bin/activate

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GST_PLUGIN_PATH="/usr/lib/gstreamer-1.0:/usr/local/lib/gstreamer-1.0"

# Detect system and set additional variables
if [ -f "/etc/nv_tegra_release" ]; then
    # Jetson specific settings
    export CUDA_VISIBLE_DEVICES=0
    export GST_GL_API=gles2
    export GST_GL_PLATFORM=egl
    echo -e "${GREEN}✅ Jetson environment configured${NC}"
elif [ -f "/etc/rpi-issue" ] || [ -f "/proc/device-tree/model" ] && grep -q "Raspberry Pi" /proc/device-tree/model; then
    # Raspberry Pi specific settings
    export GST_GL_API=gles2
    export GST_GL_PLATFORM=egl
    echo -e "${GREEN}✅ Raspberry Pi environment configured${NC}"
fi

echo -e "${GREEN}✅ Vision server environment activated${NC}"
echo -e "${BLUE}🐍 Python: $(which python)${NC}"
echo -e "${BLUE}📦 Pip: $(which pip)${NC}"
echo ""
echo -e "${GREEN}Ready to run vision server!${NC}"
echo -e "${BLUE}Example: python vision_server.py --device-id device-01${NC}"
EOF

chmod +x activate_env.sh
echo -e "${GREEN}✅ Activation script created${NC}"

# Create logs directory
mkdir -p logs

# Test installation
echo -e "${YELLOW}🧪 Testing installation...${NC}"

python3 -c "
import sys
print(f'Python version: {sys.version}')

import numpy as np
print(f'NumPy version: {np.__version__}')

import gi
print('PyGObject imported successfully')

import yaml
print('PyYAML imported successfully')

print('✅ All dependencies working correctly')
"

# Test GStreamer
if command_exists gst-launch-1.0; then
    echo -e "${GREEN}✅ GStreamer is available${NC}"
else
    echo -e "${YELLOW}⚠️  GStreamer not found in PATH${NC}"
fi

# Test v4l2-utils
if command_exists v4l2-ctl; then
    echo -e "${GREEN}✅ v4l2-utils is available${NC}"
else
    echo -e "${YELLOW}⚠️  v4l2-utils not found in PATH${NC}"
fi

echo -e "${GREEN}✅ Vision server setup completed successfully!${NC}"
echo ""
echo -e "${BLUE}📋 Next steps:${NC}"
echo -e "${BLUE}1. Activate environment: source activate_env.sh${NC}"
echo -e "${BLUE}2. Run vision server: python vision_server.py --device-id device-01${NC}"
echo -e "${BLUE}3. Check camera devices: v4l2-ctl --list-devices${NC}"
echo ""
echo -e "${GREEN}🎉 Vision server is ready!${NC}" 