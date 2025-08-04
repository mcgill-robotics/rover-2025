# Teleop System

This directory contains the complete teleop (teleoperation) system for the rover, including camera management, drive data processing, web services, and testing utilities.

## 🚀 **Quick Start**

### **1. Set Up Python Environment**

Create and activate the consolidated Python environment:

```bash
cd teleop

# Create the virtual environment and install all dependencies
./setup_env.sh

# Activate the environment (do this every time you work on teleop)
source ./activate_env.sh
```

### **2. Run the System**

With the environment activated, you can run any part of the system:

```bash
# Camera system
cd services/camera
python3 central_backend.py

# Mock testing
cd tests
python3 mock_jetson_server.py
python3 mock_ros_drive_data.py

# Frontend (separate terminal)
cd robot-controller-ui
npm run dev
```

## 📁 **Directory Structure**

```
teleop/
├── venv/                    # Python virtual environment (auto-created)
├── requirements.txt         # Combined Python dependencies
├── setup_env.sh            # Environment setup script
├── activate_env.sh         # Environment activation script (auto-created)
├── README.md               # This file
│
├── services/               # Backend services
│   ├── camera/            # Camera management system
│   │   ├── central_backend.py    # Main camera orchestrator
│   │   ├── jetson_server.py      # Jetson device service
│   │   └── ...
│   └── ros/               # ROS integration services
│       ├── ros_manager.py        # ROS data manager
│       └── ...
│
├── tests/                 # Testing and simulation
│   ├── mock_jetson_server.py     # Camera system testing
│   ├── mock_ros_drive_data.py    # Drive system testing
│   ├── full_integration_test.py  # Complete system tests
│   └── README.md                 # Testing guide
│
└── robot-controller-ui/   # Frontend React application
    ├── src/
    ├── package.json
    └── ...
```

## 🐍 **Python Environment Management**

### **Unified Dependencies**

All Python dependencies for the teleop system are now consolidated in `requirements.txt`:

- **Web Services**: `fastapi`, `uvicorn`, `aiohttp`, `websockets`
- **Computer Vision**: `opencv-python`, `numpy`
- **Testing**: `pytest`, `unittest2`
- **Utilities**: `python-dateutil`, `asyncio`

**Note on ROS2**: ROS2 packages (`rclpy`, `std_msgs`, `geometry_msgs`) are installed via system packages, not pip. Make sure to source your ROS2 environment when using ROS features:
```bash
source /opt/ros/humble/setup.bash  # or your ROS distro
```

### **Environment Commands**

```bash
# Initial setup (run once)
./setup_env.sh

# Activate environment (run every session)
source ./activate_env.sh

# Deactivate environment
deactivate

# Recreate environment (if needed)
./setup_env.sh  # Will prompt to recreate
```

### **Adding New Dependencies**

1. Add the package to `requirements.txt`
2. Reinstall: `pip install -r requirements.txt`
3. Or install directly: `pip install package_name`

## 🎯 **System Components**

### **Camera Management System**
- **Service-based architecture**: Cameras start only when requested
- **Multi-device support**: Handles multiple Jetson devices
- **Dynamic streaming**: On-demand camera activation/deactivation
- **Centralized orchestration**: Single backend manages all devices

### **Drive Data System**
- **ROS integration**: Uses proper ROS message publishing
- **Real-time diagnostics**: Motor voltage, current, temperature
- **Multiple scenarios**: Normal, fault, low battery, overheating
- **WebSocket streaming**: Live data to frontend

### **Testing Framework**
- **Mock devices**: Simulate Jetson cameras and ROS motors
- **Integration tests**: End-to-end system validation
- **Multiple scenarios**: Test various failure conditions
- **Interactive testing**: Run alongside frontend for real-time testing

## 🔧 **Development Workflow**

### **Starting Development**

```bash
# 1. Navigate to teleop directory
cd teleop

# 2. Activate Python environment
source ./activate_env.sh

# 3. Start your service/test
cd services/camera
python3 central_backend.py
```

### **Running Tests**

```bash
# Activate environment
source ./activate_env.sh

# Run specific tests
cd tests
python3 mock_jetson_server.py --num-devices 2
python3 mock_ros_drive_data.py --scenario motor_fault

# Run integration tests
python3 full_integration_test.py --test-types full
```

### **Frontend Development**

The frontend has its own Node.js environment:

```bash
cd robot-controller-ui
npm install  # First time only
npm run dev  # Start development server
```

## 🌐 **Service Ports**

- **Camera Backend**: `localhost:8081`
- **ROS Manager**: `localhost:8082`
- **Frontend**: `localhost:3000`

## 🚨 **Troubleshooting**

### **Environment Issues**

```bash
# Recreate environment
./setup_env.sh

# Check Python path
which python  # Should point to teleop/venv/bin/python

# Check installed packages
pip list
```

### **ROS Issues**

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash  # or your ROS distro

# Build custom messages if needed
colcon build --packages-select msg_srv_interface
source install/setup.bash
```

### **Port Conflicts**

If ports are in use, you can modify the default ports in the service configuration files.

## 📚 **Additional Resources**

- **Testing Guide**: See `tests/README.md` for comprehensive testing instructions
- **Camera System**: See `services/camera/README.md` for camera-specific documentation
- **Frontend**: See `robot-controller-ui/README.md` for frontend development

## 🎉 **Benefits of This Setup**

1. **Unified Environment**: Single virtual environment for all Python code
2. **Easy Setup**: One command to set up everything
3. **Consistent Dependencies**: No version conflicts between services
4. **Isolated**: Doesn't interfere with system Python or other projects
5. **Portable**: Easy to replicate on different machines
6. **Version Controlled**: Dependencies are tracked in git

This setup ensures that anyone can quickly get the teleop system running with a single command, and all Python services share the same consistent environment.
