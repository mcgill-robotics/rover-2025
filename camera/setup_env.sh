#!/bin/bash

# Vision Server Setup Script
# Compatible with Jetson, Raspberry Pi, and generic Linux

set -e

# -------------------------------
# 🎨 Colors
# -------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# -------------------------------
echo -e "${BLUE}🔧 Setting up Vision Server Environment${NC}"
echo "=============================================="

# -------------------------------
# 🔍 Command & Platform Detection
# -------------------------------
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

detect_system() {
    if [ -f "/etc/nv_tegra_release" ]; then
        echo "jetson"
    elif grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
        echo "raspberry_pi"
    else
        echo "linux"
    fi
}

SYSTEM_TYPE=$(detect_system)
echo -e "${BLUE}📱 Detected system: $SYSTEM_TYPE${NC}"

# -------------------------------
# ✅ Prerequisite Checks
# -------------------------------
echo -e "${YELLOW}🔍 Checking prerequisites...${NC}"
for cmd in python3 pip3; do
    if ! command_exists $cmd; then
        echo -e "${RED}❌ $cmd is not installed${NC}"
        exit 1
    fi
done
echo -e "${GREEN}✅ Basic prerequisites check passed${NC}"

# -------------------------------
# 📦 Install System Dependencies
# -------------------------------
echo -e "${YELLOW}📦 Installing system dependencies...${NC}"
sudo apt update

COMMON_PACKAGES=(
    python3-dev python3-pip python3-venv \
    gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    v4l-utils libv4l-dev \
    python3-opencv python3-gi libgirepository1.0-dev
)

case $SYSTEM_TYPE in
  "jetson")
    echo -e "${BLUE}🤖 Installing Jetson-specific packages...${NC}"
    sudo apt install -y "${COMMON_PACKAGES[@]}" nvidia-l4t-gstreamer
    ;;
  "raspberry_pi")
    echo -e "${BLUE}🍓 Installing Raspberry Pi-specific packages...${NC}"
    sudo apt install -y "${COMMON_PACKAGES[@]}"
    ;;
  *)
    echo -e "${BLUE}🐧 Installing Linux packages...${NC}"
    sudo apt install -y "${COMMON_PACKAGES[@]}"
    ;;
esac

echo -e "${GREEN}✅ System dependencies installed${NC}"

# -------------------------------
# 🐍 Virtual Environment
# -------------------------------
echo -e "${YELLOW}🐍 Creating Python virtual environment...${NC}"
[ -d "venv" ] && { echo -e "${YELLOW}⚠️  Removing old virtual environment...${NC}"; rm -rf venv; }
python3 -m venv venv --system-site-packages
source venv/bin/activate

# -------------------------------
# 📦 Python Dependencies
# -------------------------------
echo -e "${YELLOW}📦 Installing Python packages...${NC}"
pip install --upgrade pip
pip install \
    'numpy>=1.23.0' \
    'PyYAML>=6.0'

echo -e "${GREEN}✅ Python dependencies installed${NC}"

# -------------------------------
# 📝 Activation Script
# -------------------------------
echo -e "${YELLOW}📝 Creating activation script...${NC}"
cat > activate_env.sh << 'EOF'
#!/bin/bash
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🔧 Activating Vision Server Environment${NC}"

[ ! -d "venv" ] && { echo -e "${RED}❌ venv missing. Run setup_env.sh first.${NC}"; exit 1; }
source venv/bin/activate

export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GST_PLUGIN_PATH="/usr/lib/gstreamer-1.0:/usr/local/lib/gstreamer-1.0"

if [ -f "/etc/nv_tegra_release" ]; then
  export CUDA_VISIBLE_DEVICES=0
  export GST_GL_API=gles2
  export GST_GL_PLATFORM=egl
  echo -e "${GREEN}✅ Jetson environment configured${NC}"
elif grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  export GST_GL_API=gles2
  export GST_GL_PLATFORM=egl
  echo -e "${GREEN}✅ Raspberry Pi environment configured${NC}"
fi

echo -e "${GREEN}✅ Environment activated${NC}"
echo -e "${BLUE}🐍 Python: $(which python)${NC}"
echo -e "${BLUE}📦 Pip: $(which pip)${NC}"
echo -e "${BLUE}🚀 To run: python vision_server.py --device-id device-01${NC}"
EOF

chmod +x activate_env.sh

# -------------------------------
# 🧪 Installation Check
# -------------------------------
echo -e "${YELLOW}🧪 Testing installation...${NC}"
python3 -c "
import sys
print(f'Python version: {sys.version}')
import numpy as np
print(f'NumPy version: {np.__version__}')
import gi
print('PyGObject OK')
import yaml
print('PyYAML OK')
print('✅ All Python dependencies OK')
"

for cmd in gst-launch-1.0 v4l2-ctl; do
  if command_exists $cmd; then
    echo -e "${GREEN}✅ $cmd is available${NC}"
  else
    echo -e "${YELLOW}⚠️  $cmd not found in PATH${NC}"
  fi
done

echo -e "${GREEN}✅ Vision server setup complete!${NC}"
echo -e "${BLUE}1: source activate_env.sh${NC}"
echo -e "${BLUE}2. Run vision server: ./run_vision_server --device-id device-01${NC}"
echo -e "${BLUE}3. Check camera devices: v4l2-ctl --list-devices${NC}"
echo ""
echo -e "${GREEN}🎉 Vision server is ready!${NC}" 