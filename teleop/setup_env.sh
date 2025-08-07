#!/bin/bash

# Teleop System Setup Script
# Comprehensive setup for the Rover 2025 teleop system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Setting up Teleop System Environment${NC}"
echo "================================================"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to detect system type
detect_system() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "macos"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "linux"
    else
        echo "unknown"
    fi
}

SYSTEM_TYPE=$(detect_system)
echo -e "${BLUE}📱 Detected system: $SYSTEM_TYPE${NC}"

# Check prerequisites
echo -e "${YELLOW}🔍 Checking prerequisites...${NC}"

if ! command_exists python3; then
    echo -e "${RED}❌ Python 3 is not installed${NC}"
    echo -e "${YELLOW}Please install Python 3.8 or higher${NC}"
    exit 1
fi

if ! command_exists pip3; then
    echo -e "${RED}❌ pip3 is not installed${NC}"
    echo -e "${YELLOW}Please install pip3${NC}"
    exit 1
fi

if ! command_exists node; then
    echo -e "${RED}❌ Node.js is not installed${NC}"
    echo -e "${YELLOW}Please install Node.js 18 or higher${NC}"
    exit 1
fi

if ! command_exists npm; then
    echo -e "${RED}❌ npm is not installed${NC}"
    echo -e "${YELLOW}Please install npm${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Basic prerequisites check passed${NC}"

# Install system dependencies based on platform
echo -e "${YELLOW}📦 Installing system dependencies...${NC}"

case $SYSTEM_TYPE in
    "macos")
        echo -e "${BLUE}🍎 Installing macOS dependencies...${NC}"
        
        # Check if Homebrew is installed
        if ! command_exists brew; then
            echo -e "${RED}❌ Homebrew is not installed${NC}"
            echo -e "${YELLOW}Please install Homebrew first:${NC}"
            echo "   /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
            exit 1
        fi
        
        # Install core dependencies
        brew install \
            python@3.11 \
            node@18 \
            cmake \
            pkg-config \
            openssl \
            git \
            wget \
            curl \
            unzip
        
        # Install GStreamer for camera features
        brew install \
            gstreamer \
            gst-plugins-base \
            gst-plugins-good \
            gst-plugins-bad \
            gst-plugins-ugly \
            gst-libav
        
        # Install PyGObject
        brew install \
            pygobject3 \
            gtk+3 \
            gobject-introspection
        
        echo -e "${GREEN}✅ macOS dependencies installed${NC}"
        ;;
        
    "linux")
        echo -e "${BLUE}🐧 Installing Linux dependencies...${NC}"
        
        # Detect package manager
        if command_exists apt; then
            # Debian/Ubuntu
            sudo apt update
            sudo apt install -y \
                python3-dev \
                python3-pip \
                python3-venv \
                nodejs \
                npm \
                build-essential \
                cmake \
                pkg-config \
                libssl-dev \
                libffi-dev \
                git \
                wget \
                curl \
                unzip \
                vim \
                nano
            
            # Install GStreamer for camera features
            sudo apt install -y \
                gstreamer1.0-tools \
                gstreamer1.0-plugins-base \
                gstreamer1.0-plugins-good \
                gstreamer1.0-plugins-bad \
                gstreamer1.0-plugins-ugly \
                gstreamer1.0-libav \
                gstreamer1.0-dev \
                libgstreamer1.0-dev \
                libgstreamer-plugins-base1.0-dev \
                libgstreamer-plugins-bad1.0-dev
            
            # Install PyGObject
            sudo apt install -y \
                python3-gi \
                python3-gi-cairo \
                gir1.2-gstreamer-1.0 \
                gir1.2-gtk-3.0 \
                libgirepository1.0-dev \
                libcairo2-dev \
                libpango1.0-dev \
                libatk1.0-dev \
                libgdk-pixbuf2.0-dev \
                libgtk-3-dev
            
            # Install Docker for TileServer
            if ! command_exists docker; then
                echo -e "${YELLOW}🐳 Installing Docker...${NC}"
                curl -fsSL https://get.docker.com -o get-docker.sh
                sudo sh get-docker.sh
                sudo usermod -aG docker $USER
                rm get-docker.sh
                echo -e "${GREEN}✅ Docker installed${NC}"
            fi
            
        elif command_exists yum; then
            # CentOS/RHEL/Fedora
            sudo yum groupinstall -y "Development Tools"
            sudo yum install -y \
                python3-devel \
                python3-pip \
                nodejs \
                npm \
                cmake \
                pkg-config \
                openssl-devel \
                libffi-devel \
                git \
                wget \
                curl \
                unzip \
                gstreamer1-plugins-base \
                gstreamer1-plugins-good \
                gstreamer1-plugins-bad \
                gstreamer1-plugins-ugly \
                gstreamer1-libav \
                gstreamer1-devel \
                python3-gobject \
                gtk3-devel \
                cairo-devel \
                pango-devel \
                atk-devel \
                gdk-pixbuf2-devel
        elif command_exists pacman; then
            # Arch Linux
            sudo pacman -Syu
            sudo pacman -S \
                base-devel \
                python \
                python-pip \
                nodejs \
                npm \
                cmake \
                pkg-config \
                openssl \
                libffi \
                git \
                wget \
                curl \
                unzip \
                gstreamer \
                gst-plugins-base \
                gst-plugins-good \
                gst-plugins-bad \
                gst-plugins-ugly \
                gst-libav \
                python-gobject \
                gtk3 \
                cairo \
                pango \
                atk \
                gdk-pixbuf2 \
                docker
        else
            echo -e "${RED}❌ Unsupported Linux distribution${NC}"
            echo -e "${YELLOW}Please install dependencies manually${NC}"
            exit 1
        fi
        
        echo -e "${GREEN}✅ Linux dependencies installed${NC}"
        ;;
        
    "unknown")
        echo -e "${YELLOW}⚠️  Unknown system type. Installing basic dependencies only.${NC}"
        ;;
esac

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

# Install Python dependencies
echo -e "${YELLOW}📦 Installing Python dependencies...${NC}"

# Install core dependencies
pip install \
    fastapi>=0.100.0 \
    uvicorn>=0.20.0 \
    aiohttp>=3.8.0 \
    aiohttp-cors>=0.7.0 \
    websockets>=11.0.0 \
    flask>=2.3.0 \
    flask-cors>=4.0.0 \
    numpy>=1.23.0 \
    opencv-contrib-python==4.8.1.78 \
    PyGObject>=3.42.0 \
    PyYAML>=6.0 \
    python-dateutil>=2.8.0 \
    requests>=2.28.0 \
    python-multipart>=0.0.6

# Install development and testing dependencies
pip install \
    pytest>=7.0.0 \
    pytest-asyncio>=0.21.0 \
    pytest-cov>=4.0.0 \
    pytest-timeout>=2.0.0 \
    selenium>=4.0.0 \
    black>=23.0.0 \
    flake8>=6.0.0 \
    mypy>=1.0.0

echo -e "${GREEN}✅ Python dependencies installed${NC}"

# Install Node.js dependencies for React UI
echo -e "${YELLOW}📦 Installing Node.js dependencies...${NC}"

cd robot-controller-ui
npm install
cd ..

echo -e "${GREEN}✅ Node.js dependencies installed${NC}"

# Create activation script
echo -e "${YELLOW}📝 Creating activation script...${NC}"

cat > activate_env.sh << 'EOF'
#!/bin/bash

# Teleop System Environment Activation Script
# Activates the Python virtual environment and sets up environment variables

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
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
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export PYTHONPATH="${PYTHONPATH}:$(pwd)/services"

# Check ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS environment not sourced. Some features may be limited.${NC}"
    echo -e "${YELLOW}   To enable ROS features, run: source /opt/ros/humble/setup.bash${NC}"
else
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
EOF

chmod +x activate_env.sh
echo -e "${GREEN}✅ Activation script created${NC}"

# Create logs directory
mkdir -p logs

# Test installation
echo -e "${YELLOW}🧪 Testing installation...${NC}"

# Test Python imports
python3 -c "
import sys
print(f'Python version: {sys.version}')

import fastapi
print(f'FastAPI version: {fastapi.__version__}')

import flask
print(f'Flask version: {flask.__version__}')

import numpy as np
print(f'NumPy version: {np.__version__}')

import cv2
print(f'OpenCV version: {cv2.__version__}')

import gi
print('PyGObject imported successfully')

import yaml
print('PyYAML imported successfully')

print('✅ All core dependencies working correctly')
"

# Test Node.js
if command_exists node; then
    echo -e "${GREEN}✅ Node.js is available${NC}"
    node --version
else
    echo -e "${YELLOW}⚠️  Node.js not found in PATH${NC}"
fi

# Test npm
if command_exists npm; then
    echo -e "${GREEN}✅ npm is available${NC}"
    npm --version
else
    echo -e "${YELLOW}⚠️  npm not found in PATH${NC}"
fi

# Test GStreamer
if command_exists gst-launch-1.0; then
    echo -e "${GREEN}✅ GStreamer is available${NC}"
else
    echo -e "${YELLOW}⚠️  GStreamer not found in PATH${NC}"
fi

# Test Docker
if command_exists docker; then
    echo -e "${GREEN}✅ Docker is available${NC}"
    docker --version
else
    echo -e "${YELLOW}⚠️  Docker not found in PATH${NC}"
fi

# Check ROS2
if command_exists ros2; then
    echo -e "${GREEN}✅ ROS2 is available${NC}"
    ros2 --version
else
    echo -e "${YELLOW}⚠️  ROS2 not found in PATH${NC}"
    echo -e "${YELLOW}   Some features may be limited without ROS2${NC}"
fi

echo -e "${GREEN}✅ Teleop system setup completed successfully!${NC}"
echo ""
echo -e "${BLUE}📋 Next steps:${NC}"
echo -e "${BLUE}1. Activate environment: source activate_env.sh${NC}"
echo -e "${BLUE}2. Start complete system: ./start-teleop-system.sh${NC}"
echo -e "${BLUE}3. Test individual services: cd services && ./test_service.sh [camera|gps|ros]${NC}"
echo -e "${BLUE}4. Start UI only: cd robot-controller-ui && npm run dev${NC}"
echo ""
echo -e "${GREEN}🎉 Teleop system is ready for development!${NC}"
