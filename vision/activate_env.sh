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
