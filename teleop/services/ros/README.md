# ROS Services

This folder contains the ROS (Robot Operating System) integration services for the Rover 2025 teleop system.

## Overview

The ROS Services provide communication between the teleop system and the rover's hardware. They handle ROS topic publishing/subscribing, data conversion, and provide a web API for the React UI to control the rover.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   React UI      │    │   ROS Manager   │    │   ROS System    │
│   (Frontend)    │◄──►│   (Web API)     │◄──►│   (Hardware)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Control Commands│    │  ROS Bridge     │    │  Motor Control  │
│  (Port 3000)    │    │  (Port 8082)    │    │  (ROS Topics)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Core Files
- **`ros_manager.py`** - Main ROS bridge and web API
- **`utils/ros_bridge.py`** - ROS communication utilities

### GPS Services (`gps/`)
- **`subscriber/gps_data_subscriber.py`** - GPS data collection for drive control
- **`README.md`** - GPS service documentation

### Drive Services (`drive/`)
- **`subscriber/drive_data_subscriber.py`** - Drive data collection
- **`README.md`** - Drive service documentation

### Testing (`tests/`)
- **`test_gps_service.py`** - GPS service tests
- **`test_mock_gps_publisher.py`** - Mock GPS publisher for testing
- **`test_mock_drive_publisher.py`** - Mock drive publisher for testing

## Features

### ROS Communication
- ✅ **Topic Publishing** - Send control commands to rover
- ✅ **Topic Subscribing** - Receive sensor data from rover
- ✅ **Message Conversion** - Convert between ROS and web formats
- ✅ **Real-time Data** - Low-latency communication

### Web API
- ✅ **REST Endpoints** - HTTP API for UI communication
- ✅ **WebSocket Support** - Real-time data streaming
- ✅ **CORS Support** - Cross-origin requests
- ✅ **Health Monitoring** - Service status tracking

### Data Types
- ✅ **Drive Commands** - Motor control and movement
- ✅ **Arm Commands** - Robotic arm manipulation
- ✅ **GPS Data** - Position and orientation
- ✅ **IMU Data** - Motion and orientation sensors
- ✅ **Motor Diagnostics** - Motor status and health

## Integration with Teleop System

### React UI (`robot-controller-ui/`)
- **`useDriveData.ts`** - Drive data management hook
- **`sections/drive/`** - Drive control components
- **`sections/arm/`** - Arm control components
- **WebSocket connections** - Real-time data streaming

### GPS Services (`services/gps/`)
- **Standalone GPS service** - For mapping interface
- **ROS GPS subscriber** - For drive control (in `services/ros/gps/`)
- **Different purposes** - Mapping vs. drive control

### Service Manager (`services/`)
- **`service_manager.py`** - Manages ROS service lifecycle
- **`service_config.yml`** - Central configuration
- **Health monitoring** - Service status tracking

## Configuration

### Service Configuration (`service_config.yml`)
```yaml
ros_manager:
  port: 8082
  enabled: true
  config:
    ros_topics:
      gps: "/gps/fix"
      imu: "/imu/data"
      drive: "/drive/cmd_vel"
      arm: "/arm/cmd_vel"
    web_api:
      cors_origins: ["*"]
      max_connections: 100
```

### ROS Topics
- **`/gps/fix`** - GPS position data (NavSatFix)
- **`/imu/data`** - IMU sensor data (Imu)
- **`/drive/cmd_vel`** - Drive velocity commands (Twist)
- **`/arm/cmd_vel`** - Arm velocity commands (Twist)
- **`/drive/status`** - Motor status and diagnostics

## API Endpoints

### REST Endpoints
- **`GET /api/health`** - Service health check
- **`GET /api/ros/status`** - ROS connection status
- **`GET /api/ros/topics`** - Available ROS topics
- **`POST /api/drive/command`** - Send drive command
- **`POST /api/arm/command`** - Send arm command

### WebSocket Endpoints
- **`/ws/ros/data`** - Real-time ROS data stream
- **`/ws/drive/status`** - Drive status updates
- **`/ws/arm/status`** - Arm status updates

## Usage

### Starting the Service
```bash
# Via service manager (recommended)
cd services
./start_services.sh --mode ros-only

# Direct startup
python3 ros/ros_manager.py
```

### ROS Environment Setup
```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Check ROS topics
ros2 topic list
ros2 topic echo /gps/fix
```

### Integration with UI
The React UI automatically connects to the ROS manager and provides:
- Real-time drive control
- Arm manipulation interface
- Motor diagnostics display
- GPS data for navigation

## File Structure

```
services/ros/
├── ros_manager.py                    # Main ROS bridge
├── utils/
│   └── ros_bridge.py                # ROS utilities
├── gps/                             # GPS services for drive control
│   ├── subscriber/
│   │   └── gps_data_subscriber.py   # GPS data subscriber
│   └── README.md                    # GPS service docs
├── drive/                           # Drive services
│   ├── subscriber/
│   │   └── drive_data_subscriber.py # Drive data subscriber
│   └── README.md                    # Drive service docs
├── tests/                           # ROS service tests
│   ├── test_gps_service.py         # GPS service tests
│   ├── test_mock_gps_publisher.py  # Mock GPS publisher
│   └── test_mock_drive_publisher.py # Mock drive publisher
└── README.md                        # This documentation
```

## Dependencies

### System Dependencies
- **ROS2** - Robot Operating System
- **Python 3.8+** - Python runtime
- **rclpy** - ROS2 Python client

### Python Dependencies
- **rclpy** - ROS2 Python client
- **sensor_msgs** - ROS sensor messages
- **geometry_msgs** - ROS geometry messages
- **aiohttp** - Async HTTP server
- **websockets** - WebSocket support

## ROS Messages

### Drive Commands (Twist)
```python
geometry_msgs.msg.Twist:
  linear:
    x: float  # Forward/backward velocity
    y: float  # Left/right velocity
    z: float  # Up/down velocity
  angular:
    x: float  # Roll rate
    y: float  # Pitch rate
    z: float  # Yaw rate (turning)
```

### GPS Data (NavSatFix)
```python
sensor_msgs.msg.NavSatFix:
  header: Header
  status: NavSatStatus
  latitude: float64
  longitude: float64
  altitude: float64
  position_covariance: float64[9]
  position_covariance_type: uint8
```

### IMU Data (Imu)
```python
sensor_msgs.msg.Imu:
  header: Header
  orientation: geometry_msgs.msg.Quaternion
  angular_velocity: geometry_msgs.msg.Vector3
  linear_acceleration: geometry_msgs.msg.Vector3
```

## Testing

### Running Tests
```bash
# Run all ROS tests
cd services/ros/tests
python3 -m pytest

# Run specific test
python3 -m pytest test_gps_service.py -v
```

### Mock Publishers
The test suite includes mock publishers for testing without actual ROS hardware:
- **`test_mock_gps_publisher.py`** - Simulates GPS data
- **`test_mock_drive_publisher.py`** - Simulates drive data

## Troubleshooting

### Common Issues
1. **ROS not found** - Check ROS installation and environment
2. **Topics not publishing** - Verify ROS nodes are running
3. **Connection refused** - Check ROS master is running
4. **Message type errors** - Verify message imports

### Debug Commands
```bash
# Check ROS service status
curl http://localhost:8082/api/health

# Check ROS topics
ros2 topic list
ros2 topic info /gps/fix

# Check ROS nodes
ros2 node list
ros2 node info /ros_manager

# Monitor ROS data
ros2 topic echo /gps/fix
ros2 topic echo /drive/cmd_vel
```

## Integration Notes

This ROS service is designed to work seamlessly with:
- **React UI** in the `robot-controller-ui/` folder
- **GPS services** in the `services/gps/` folder (for mapping)
- **Service manager** in the parent `services/` folder

The service provides the bridge between the web-based teleop interface and the rover's ROS-based hardware control system.

---

**Note**: This is the ROS integration component of the teleop system. For standalone GPS services, see the `services/gps/` folder documentation. 