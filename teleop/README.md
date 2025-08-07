# Rover 2025 Teleop System

A comprehensive teleoperation system for the Rover 2025 project, featuring a modern React UI, unified backend services, offline mapping capabilities, and full ROS integration.

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Rover 2025 Teleop System                 │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
        ▼             ▼             ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│ React UI    │ │ Services    │ │ Offline     │
│ (Port 3000) │ │ (Ports      │ │ Mapping     │
│             │ │ 8082-8083)  │ │ (Port 8080) │
└─────────────┘ └─────────────┘ └─────────────┘
        │             │             │
        ▼             ▼             ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│ Drive       │ │ ROS Manager │ │ TileServer  │
│ Arm         │ │ GPS Service │ │ MapLibre    │
│ Mapping     │ │ Web APIs    │ │ OSM Tiles   │
│ Status      │ │ Health      │ │ Offline     │
└─────────────┘ └─────────────┘ └─────────────┘
```

## 📁 Directory Structure

```
teleop/
├── README.md                    # This file - Central documentation
├── setup_env.sh                # Environment setup script
├── activate_env.sh             # Environment activation script
├── start-teleop-system.sh      # Complete system startup
├── venv/                       # Python virtual environment
├── tests/                      # Integration tests only
│   ├── integration_test.py     # Complete system test
│   └── __init__.py
├── services/                   # Backend services
│   ├── service_manager.py     # Unified service manager
│   ├── service_config.yml     # Service configuration (YAML)
│   ├── start_services.sh      # Services startup
│   ├── tests/                 # Individual service tests
│   │   └── run_tests.sh       # Service test runner
│   ├── ros/                   # ROS integration
│   │   ├── gps/              # ROS GPS subscriber (for drive control)
│   │   │   ├── README.md     # GPS service documentation
│   │   │   └── subscriber/   # GPS subscriber code
│   │   ├── drive/            # ROS drive data subscriber
│   │   │   ├── README.md     # Drive service documentation
│   │   │   └── subscriber/   # Drive subscriber code
│   │   ├── tests/            # ROS test files
│   │   │   ├── test_gps_service.py
│   │   │   ├── test_mock_gps_publisher.py
│   │   │   └── test_mock_drive_publisher.py
│   │   └── utils/            # ROS utilities
│   ├── gps/                   # GPS & offline mapping services
│   │   ├── gps_service.py     # Standalone GPS service (for mapping)
│   │   ├── gps_api.py         # Flask GPS API
│   │   ├── docker-compose.tileserver.yml
│   │   ├── README.md          # Comprehensive GPS documentation
│   │   └── download-scripts/  # Map tile downloaders
│   │       ├── download-mcgill-basic.sh      # Basic McGill campus
│   │       ├── download-mcgill-complete.sh   # Complete McGill coverage
│   │       ├── download-drumheller-tiles.sh  # Basic Drumheller
│   │       ├── download-drumheller-complete.sh # Complete Drumheller
│   │       └── download-custom-area.sh       # Any custom area
│   └── camera/                # Camera backend service
│       ├── camera_service.py  # Multi-camera backend (renamed)
│       ├── aruco_detector.py  # ArUco marker detection
│       └── gstreamer_reader.py # GStreamer video processing
└── robot-controller-ui/        # React frontend
    ├── package.json
    ├── src/
    │   ├── app/               # Next.js pages
    │   ├── components/        # React components
    │   ├── hooks/            # Custom hooks
    │   └── store/            # State management
    └── public/               # Static assets

**Note**: Jetson/vision processing components have been moved to `rover-2025/vision/`:
```
rover-2025/vision/            # Jetson/vision processing
├── vision_server.py          # Multi-camera vision server (renamed)
├── gstreamer_pipeline.py     # GStreamer pipeline manager with Python bindings
├── config.py                 # Configuration loader
├── config.yml                # Vision configuration (YAML)
├── setup_env.sh             # Lightweight environment setup
└── README.md                 # Vision system documentation
```

## 🚀 Quick Start

### 1. Environment Setup

```bash
# Navigate to teleop directory
cd rover-2025/teleop

# Setup Python environment
./setup_env.sh

# Activate environment
source activate_env.sh
```

### 2. Start Complete System

```bash
# Start everything with one command
./start-teleop-system.sh

# Or start specific services using modes
cd services
./start_system.sh --mode basic      # ROS + GPS only
./start_system.sh --mode mapping    # ROS + GPS + TileServer
./start_system.sh --mode camera     # ROS + GPS + Camera service
./start_system.sh --mode ros-only   # ROS Manager only
./start_system.sh --mode all        # All services (default)
```

This comprehensive script will:
- ✅ Check prerequisites (Docker, Node.js, Python)
- ✅ Download map tiles if needed
- ✅ Start TileServer-GL (Port 8080)
- ✅ Start Unified Service Manager (Port 8083)
- ✅ Start ROS Manager (Port 8082)
- ✅ Start GPS Service (Port 5001)
- ✅ Start React UI (Port 3000)
- ✅ Provide graceful shutdown with Ctrl+C

### 3. Individual Service Testing
```bash
# Start specific services for testing
cd services

# Start camera service only
./test_service.sh camera

# Start GPS service only
./test_service.sh gps

# Start ROS manager only
./test_service.sh ros

# Start TileServer only
./test_service.sh tileserver

# Or use individual scripts
cd camera && ./start_camera_service.sh
cd gps && ./start_gps_service.sh
cd ros && ./start_ros_manager.sh
```

### 4. Access the System

- **Main UI**: http://localhost:3000
- **Drive Control**: http://localhost:3000/drive
- **Arm Control**: http://localhost:3000/arm
- **Offline Navigation**: http://localhost:3000/navigation

## 🎮 Features

### Drive Control
- Real-time drive control with gamepad support
- Motor diagnostics and status monitoring
- Speed control and safety limits
- Camera feed integration
- **GPS integration** via ROS GPS subscriber

### Arm Control
- 6-DOF robotic arm control
- Inverse kinematics
- Joint limits and safety
- Visual arm representation

### Offline Mapping
- **No internet required** - Complete offline operation
- Real-time GPS tracking with rover position
- Waypoint management and navigation
- Multiple map styles (street, satellite, terrain)
- Export/import waypoint data
- **Standalone GPS service** with Flask API

## 🔧 GPS Services Architecture

The system has two GPS services serving different purposes:

### 1. ROS GPS Subscriber (`services/ros/gps/`)
- **Purpose**: Integrates with ROS manager for drive control interface
- **Topics**: `/gps_coordinates`, `/imu_data`
- **Integration**: Part of the ROS bridge system
- **Use Case**: Drive control, navigation, real-time rover tracking

### 2. Standalone GPS Service (`services/gps/`)
- **Purpose**: Backend service for the mapping interface
- **Topics**: `/gps/fix`, `/imu/data`, `/cmd_vel`
- **Integration**: Flask API for mapping frontend
- **Use Case**: Offline mapping, waypoint management, GPS visualization
- **Documentation**: See `services/gps/README.md` for comprehensive setup and technical details

## 🧹 ROS Services Organization

The ROS services are organized with consistent structure:

### ROS GPS (`services/ros/gps/`)
- **README.md** - Service documentation
- **subscriber/** - GPS subscriber code
- **Tests** - Located in `services/ros/tests/`

### ROS Drive (`services/ros/drive/`)
- **README.md** - Service documentation  
- **subscriber/** - Drive subscriber code
- **Tests** - Located in `services/ros/tests/`

### ROS Tests (`services/ros/tests/`)
- **test_gps_service.py** - GPS service tests
- **test_mock_gps_publisher.py** - Mock GPS data publisher
- **test_mock_drive_publisher.py** - Mock drive data publisher

### System Monitoring
- Real-time system health monitoring
- Service status and restart capabilities
- Performance metrics and logging
- Error handling and recovery

### Vision System

The vision system has been significantly improved with native GStreamer Python bindings:

#### GStreamer Pipeline Manager
- Native Python bindings (gi) for better performance
- Dynamic property updates (bitrate, FPS) without pipeline restart
- Automatic camera format detection (MJPG, YUYV, RAW)
- Robust error handling and recovery
- Proper resource cleanup

#### Multi-Dictionary ArUco Detection
- Support for multiple marker dictionaries (4x4, 5x5, 6x6)
- Color-coded visualization by marker size
- Improved marker tracking and identification
- Enhanced overlay with size and ID information

#### Camera Service API
- Dedicated FastAPI endpoints for camera control
- Dynamic bitrate adjustment via UI slider
- Improved error handling and status reporting
- Clean separation of API and core logic

#### Configuration
All camera and vision settings are now in YAML:
```yaml
gstreamer_config:
  # Network settings
  max_udp_packet_size: 65507
  udp_buffer_size: 1048576
  rtp_buffer_size: 1048576
  
  # H.264 encoding
  h264_bitrate: 2048
  h264_tune: "zerolatency"
  h264_profile: "baseline"
  h264_level: "4.1"
  
  # Frame settings
  max_frame_size: 1920x1080
  frame_buffer_size: 10
  default_fps: 30
  min_fps: 5
  max_fps: 60
```

## 🔧 Configuration

### Service Configuration

Edit `services/service_config.yml` to customize services:

```yml
services:
  ros_manager:
    port: 8082
    enabled: true
  gps_service:
    port: 5001
    enabled: true
  tileserver:
    port: 8080
    enabled: true
    docker_compose_file: "gps/docker-compose.tileserver.yml"
    config:
      options: // TileServer options
      styles: // Map styles
      data: Map data sources
  "monitoring": {
    "health_check_interval": 30,
    "restart_failed_services": true
  }
}
```

**Note**: TileServer configuration is now integrated into the main service config for centralized management.

### Environment Variables

```bash
# ROS environment
export ROS_DISTRO=humble
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# Logging
export ROS_LOG_LEVEL=INFO
```

## 🗺️ Offline Mapping

The system includes comprehensive offline mapping capabilities with pre-configured scripts for common areas. For detailed technical documentation, see `services/gps/README.md`.

### Quick Setup

```bash
# Navigate to download scripts
cd services/gps/download-scripts

# McGill campus (quick setup)
./download-mcgill-basic.sh

# Drumheller Badlands (desert testing)
./download-drumheller-tiles.sh

# Any custom area
./download-custom-area.sh --bounds "45.5,-73.6,45.6,-73.5" --name "montreal"
```

### Available Areas

#### 🏫 McGill University Campus
- **Basic**: `download-mcgill-basic.sh` - Quick setup for campus testing
- **Complete**: `download-mcgill-complete.sh` - Full campus coverage with real tiles
- **Coverage**: Montreal downtown area, McGill campus, surrounding streets
- **Use Case**: Urban campus navigation and testing

#### 🏜️ Drumheller Badlands (Desert Testing)
- **Basic**: `download-drumheller-tiles.sh` - Quick setup for desert testing
- **Complete**: `download-drumheller-complete.sh` - Full badlands coverage
- **Coverage**: Canadian Badlands, Dinosaur Provincial Park, Red Deer River
- **Use Case**: Off-road desert navigation without WiFi

#### 🌍 Custom Areas
- **Generic**: `download-custom-area.sh` - Download any area worldwide
- **Usage**: `./download-custom-area.sh --bounds "lat1,lon1,lat2,lon2" --name "area_name"`
- **Examples**: Montreal, NYC, Toronto, Vancouver, any custom location

### Map Features
- ✅ **Offline Operation** - No internet required
- ✅ **Multiple Styles** - OSM, Satellite, Terrain views
- ✅ **GPS Tracking** - Real-time position on offline maps
- ✅ **Waypoint Management** - Mark and navigate to points
- ✅ **Route Planning** - Plan routes on offline maps
- ✅ **High Detail** - Configurable zoom levels (10-18)

### File Sizes
| Area Type | Zoom Levels | Size | Time |
|-----------|-------------|------|------|
| Small campus | 12-16 | ~500MB | 10-20 min |
| Medium city | 10-16 | ~2GB | 20-40 min |
| Large area | 10-18 | ~5GB | 30-60 min |

## 🧪 Testing

### Integration Tests

```bash
# Run complete system integration test
python3 tests/integration_test.py
```

Tests include:
- ✅ Service Manager API
- ✅ ROS Manager API
- ✅ GPS Service API
- ✅ React UI pages
- ✅ UI-API integration
- ✅ Mapping functionality
- ✅ System health

### Service Tests

```bash
# Run individual service tests
cd services
./run_tests.sh
```

## 📊 API Documentation

### Service Manager API (Port 8083)

```bash
# Get all service status
curl http://localhost:8083/api/status

# Restart a service
curl -X POST http://localhost:8083/api/restart/ros_manager

# Get configuration
curl http://localhost:8083/api/config
```

### ROS Manager API (Port 8082)

```bash
# Drive diagnostics
curl http://localhost:8082/api/drive/diagnostics

# Drive speeds
curl http://localhost:8082/api/drive/speeds

# Drive status
curl http://localhost:8082/api/drive/status

# GPS coordinates
curl http://localhost:8082/api/gps/coordinates
```

### GPS Service API (Port 5001)

```bash
# GPS data
curl http://localhost:5001/api/gps/data

# GPS status
curl http://localhost:5001/api/gps/status

# Stream GPS data
curl http://localhost:5001/api/gps/stream
```

## 🗺️ Offline Mapping Setup

### 1. Download Map Tiles

```bash
cd services/gps
./download-map-tiles.sh
```

### 2. Start TileServer

```bash
docker-compose -f docker-compose.tileserver.yml up -d
```

### 3. Access Mapping

Navigate to http://localhost:3000/mapping

## 🛠️ Development

### Adding New Features

1. **Backend Services**: Add to `services/` directory
2. **Frontend Components**: Add to `robot-controller-ui/src/components/`
3. **API Endpoints**: Add to appropriate service
4. **Tests**: Add to `services/tests/` for services, `tests/` for integration

### Code Structure

- **Services**: Python with async/await patterns
- **Frontend**: React with TypeScript and Next.js
- **State Management**: Zustand for client state
- **Styling**: Tailwind CSS
- **Testing**: pytest for Python, Jest for JavaScript

## 🚨 Troubleshooting

### Common Issues

#### Services Won't Start
```bash
# Check if ports are in use
lsof -i :8082
lsof -i :5001
lsof -i :8083

# Check ROS environment
echo $ROS_DISTRO
ros2 topic list
```

#### UI Not Loading
```bash
# Check if Node.js dependencies are installed
cd robot-controller-ui
npm install --legacy-peer-deps

# Check if UI is running
curl http://localhost:3000
```

#### GPS Data Not Updating
```bash
# Check GPS service logs
tail -f services/logs/service_manager.log

# Check ROS topics
ros2 topic echo /gps/fix
ros2 topic echo /imu/data
```

#### Mapping Not Working
```bash
# Check TileServer
curl http://localhost:8080

# Check map tiles
ls -la services/mbtiles/
```

### Performance Issues

1. **High CPU Usage**: Check health check interval in config
2. **Memory Leaks**: Monitor service restarts
3. **Network Issues**: Check port availability and firewall settings

## 📚 Dependencies

### Python Packages
- `aiohttp`: Async HTTP server
- `flask`: Web framework for GPS service
- `rclpy`: ROS 2 Python client
- `opencv-contrib-python`: Computer vision
- `PyGObject`: GStreamer integration
- `pytest`: Testing framework
- `selenium`: Browser automation for tests

### Node.js Packages
- `next`: React framework
- `react`: UI library
- `maplibre-gl`: Offline mapping
- `zustand`: State management
- `lucide-react`: Icons
- `tailwindcss`: Styling

### System Requirements
- Python 3.8+
- Node.js 18+
- ROS 2 (Humble/Foxy) or ROS 1 (Noetic)
- Docker (for TileServer-GL)
- Chrome/Chromium (for tests)

## 🤝 Contributing

1. Follow existing code patterns
2. Add tests for new features
3. Update documentation
4. Run full test suite before submitting
5. Use meaningful commit messages

### Development Workflow

```bash
# 1. Setup environment
./setup_env.sh
source activate_env.sh

# 2. Start services for development
cd services
./start_services.sh

# 3. Start UI for development
cd robot-controller-ui
npm run dev

# 4. Run tests
python3 tests/integration_test.py
cd services && ./run_tests.sh
```

## 📄 License

This project uses the same license as the main Rover 2025 project.

## 🔗 Related Documentation

- [GPS Services Documentation](services/gps/README.md) - Comprehensive GPS and offline mapping documentation
- [ROS Integration](services/ros/README.md)
- [Camera Services](services/camera/README.md)
- [Vision System](../vision/README.md) - Jetson/Pi vision processing

---

**Need Help?** Check the troubleshooting section above or open an issue in the project repository.
