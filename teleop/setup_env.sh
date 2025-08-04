#!/bin/bash
# Teleop Python Environment Setup Script
# Creates and manages a Python virtual environment for the teleop system

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements.txt"

echo -e "${BLUE}🚀 Teleop Python Environment Setup${NC}"
echo "Script directory: $SCRIPT_DIR"
echo "Virtual environment: $VENV_DIR"
echo "Requirements file: $REQUIREMENTS_FILE"
echo ""

# Function to print status messages
print_status() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    print_error "Python 3 is not installed or not in PATH"
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
print_status "Found $PYTHON_VERSION"

# Check if virtual environment already exists
if [ -d "$VENV_DIR" ]; then
    print_warning "Virtual environment already exists at $VENV_DIR"
    read -p "Do you want to recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_status "Removing existing virtual environment..."
        rm -rf "$VENV_DIR"
    else
        print_status "Using existing virtual environment"
    fi
fi

# Create virtual environment if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    print_status "Creating Python virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
print_status "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
print_status "Upgrading pip..."
pip install --upgrade pip

# Install requirements
if [ -f "$REQUIREMENTS_FILE" ]; then
    print_status "Installing Python dependencies from requirements.txt..."
    
    # Check if we're on macOS and need GStreamer dependencies
    if [[ "$OSTYPE" == "darwin"* ]]; then
        print_status "Detected macOS - checking GStreamer dependencies..."
        
        # Check if brew is available
        if command -v brew &> /dev/null; then
            # Check if GStreamer is installed
            if ! brew list gstreamer &> /dev/null; then
                print_warning "GStreamer not found. Installing via brew..."
                echo "This may take a few minutes..."
                brew install pygobject3 gtk+3 gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly
            else
                print_status "GStreamer dependencies found"
            fi
        else
            print_warning "Homebrew not found. You may need to install GStreamer manually:"
            echo "brew install pygobject3 gtk+3 gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly"
        fi
    fi
    
    pip install -r "$REQUIREMENTS_FILE"
else
    print_error "Requirements file not found: $REQUIREMENTS_FILE"
    exit 1
fi

# Create activation script
ACTIVATE_SCRIPT="$SCRIPT_DIR/activate_env.sh"
cat > "$ACTIVATE_SCRIPT" << 'EOF'
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
EOF

chmod +x "$ACTIVATE_SCRIPT"

# Create deactivation reminder
echo ""
print_status "Environment setup complete!"
echo ""
echo -e "${BLUE}📋 Usage Instructions:${NC}"
echo "1. To activate the environment:"
echo -e "   ${YELLOW}source ./activate_env.sh${NC}"
echo ""
echo "2. To deactivate the environment:"
echo -e "   ${YELLOW}deactivate${NC}"
echo ""
echo "3. To recreate the environment:"
echo -e "   ${YELLOW}./setup_env.sh${NC}"
echo ""
echo -e "${BLUE}📁 Directory Structure:${NC}"
echo "teleop/"
echo "├── venv/                 # Python virtual environment"
echo "├── requirements.txt      # Combined Python dependencies"
echo "├── setup_env.sh         # This setup script"
echo "├── activate_env.sh      # Environment activation script"
echo "├── services/            # Backend services"
echo "├── tests/               # Testing utilities"
echo "└── robot-controller-ui/ # Frontend application"
echo ""

# Check if ROS2 is available
echo ""
print_status "Checking ROS2 environment..."
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version 2>/dev/null || echo 'Unknown version')
    print_status "ROS2 detected: $ROS_VERSION"
    
    # Test if ROS2 Python packages are available
    if python -c "import rclpy" 2>/dev/null; then
        print_status "ROS2 Python packages are available in this environment"
    else
        print_warning "ROS2 Python packages not found in virtual environment"
        echo "💡 This is normal - ROS2 packages use system Python installation"
        echo "💡 Make sure to source your ROS2 environment before running ROS-based services:"
        echo -e "   ${YELLOW}source /opt/ros/humble/setup.bash${NC}  # or your ROS distro"
    fi
else
    print_warning "ROS2 not found in PATH. Some functionality may be limited."
    echo "💡 To install ROS2, visit: https://docs.ros.org/en/humble/Installation.html"
    echo "💡 After installation, source the environment:"
    echo -e "   ${YELLOW}source /opt/ros/humble/setup.bash${NC}  # or your ROS distro"
fi

echo ""
print_status "Environment setup complete!"
echo ""
echo -e "${BLUE}🚀 Next Steps:${NC}"
echo -e "1. ${YELLOW}source ./activate_env.sh${NC} - Activate Python environment"
echo -e "2. ${YELLOW}source /opt/ros/humble/setup.bash${NC} - Source ROS2 (if using ROS features)"
echo "3. Start your services or tests"
echo ""
print_status "Your teleop Python environment is ready to use!"
