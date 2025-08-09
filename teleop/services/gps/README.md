# GPS Services

This folder contains the GPS and offline mapping services for the Rover 2025 teleop system.

## Overview

The GPS Services provide GPS data collection, offline mapping capabilities, and map tile serving for the teleop system. This enables GPS-based navigation and mapping without requiring internet connectivity.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   GPS Device    │    │   GPS Service   │    │   React UI      │
│   (Hardware)    │───►│   (ROS/Flask)   │───►│   (Frontend)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  GPS/IMU Data   │    │  SSE Streaming  │    │  Mapping Display│
│  (ROS Topics)   │    │  (Port 5001)    │    │  (Port 3000)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Quick Setup

### 1. Environment Setup
```bash
# Navigate to the teleop directory
cd rover-2025/teleop

# Setup complete environment
./setup_env.sh

# Activate environment
source activate_env.sh
```

### 2. Download Offline Map Tiles
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

### 3. Start Complete System
```bash
# Start everything with one command
cd ../../..
./start-teleop-system.sh
```

This will start:
- TileServer-GL (Port 8080)
- GPS Service (Port 5001)
- React UI (Port 3000)

## Available Map Areas

### 🏫 McGill University Campus
- **Basic**: `download-mcgill-basic.sh` - Quick setup for campus testing
- **Complete**: `download-mcgill-complete.sh` - Full campus coverage with real tiles
- **Coverage**: Montreal downtown area, McGill campus, surrounding streets

### 🏜️ Drumheller Badlands (Desert Testing)
- **Basic**: `download-drumheller-tiles.sh` - Quick setup for desert testing
- **Complete**: `download-drumheller-complete.sh` - Full badlands coverage
- **Coverage**: Canadian Badlands, Dinosaur Provincial Park, Red Deer River

### 🌍 Custom Areas
- **Generic**: `download-custom-area.sh` - Download any area worldwide
- **Usage**: `./download-custom-area.sh --bounds "lat1,lon1,lat2,lon2" --name "area_name"`

## Key Components

### Core Files
- **`gps_service.py`** - Main GPS data service (ROS integration)
- **`gps_api.py`** - Flask API for GPS data
- **`docker-compose.tileserver.yml`** - TileServer-GL setup

### Download Scripts (`download-scripts/`)
- **`download-mcgill-basic.sh`** - Basic McGill campus tiles (quick setup)
- **`download-mcgill-complete.sh`** - Complete McGill campus coverage
- **`download-drumheller-tiles.sh`** - Basic Drumheller Badlands tiles
- **`download-drumheller-complete.sh`** - Complete Drumheller coverage
- **`download-custom-area.sh`** - Download tiles for any custom area

## Features

### GPS Data Collection
- ✅ **Real-time GPS** - Live GPS coordinate streaming
- ✅ **IMU Integration** - Orientation and movement data
- ✅ **ROS Integration** - GPS data from ROS topics
- ✅ **Server-Sent Events** - Real-time data streaming

### Offline Mapping
- ✅ **OpenStreetMap** - Open-source map data
- ✅ **TileServer-GL** - Offline map tile serving
- ✅ **MapLibre GL** - Frontend mapping library
- ✅ **No Internet Required** - Fully offline operation

### Map Management
- ✅ **Tile Downloading** - Pre-download map tiles
- ✅ **Custom Areas** - Download specific regions
- ✅ **Multiple Styles** - Satellite, terrain, street maps
- ✅ **Efficient Storage** - Compressed tile format

## Integration with Teleop System

### ROS Integration (`services/ros/`)
- **`gps_data_subscriber.py`** - GPS data from ROS topics
- **`ros_manager.py`** - ROS communication hub
- **GPS topics** - `/gps/fix`, `/imu/data`

### React UI (`robot-controller-ui/`)
- **`useGPSData.ts`** - GPS data management hook
- **`sections/mapping/`** - Mapping interface components
- **MapLibre integration** - Offline map display

### Service Manager (`services/`)
- **`service_manager.py`** - Manages GPS service lifecycle
- **`service_config.yml`** - Central configuration
- **Health monitoring** - Service status tracking

## Configuration

### Service Configuration (`service_config.yml`)
```yaml
gps_service:
  port: 5001
  enabled: true
  config:
    flask:
      host: "0.0.0.0"
      debug: false
      threaded: true
    sse:
      retry_timeout: 30000
      keep_alive_interval: 30
    gps:
      update_interval: 1.0
      coordinate_precision: 6

tileserver:
  port: 8080
  enabled: true
  docker_compose_file: "gps/docker-compose.tileserver.yml"
```

### GPS Topics Configuration
Edit `gps_service.py` to match your ROS topic names:

```python
# Default topic names
self.gps_subscription = self.create_subscription(
    NavSatFix,
    '/gps/fix',  # Change this to your GPS topic
    10
)

self.imu_subscription = self.create_subscription(
    Imu,
    '/imu/data',  # Change this to your IMU topic
    10
)
```

## API Endpoints

### REST Endpoints
- **`GET /api/health`** - Service health check
- **`GET /api/gps/data`** - Current GPS coordinates
- **`GET /api/gps/status`** - GPS device status
- **`GET /api/config`** - Get GPS configuration

### Server-Sent Events
- **`/api/gps/stream`** - Real-time GPS data stream

### Example GPS Data Response
```json
{
  "latitude": 45.5048,
  "longitude": -73.5772,
  "heading": 180.5,
  "accuracy": 5.2,
  "timestamp": 1640995200000,
  "fix_quality": 1,
  "satellites": 8
}
```

## Usage

### Accessing the Mapping Interface
1. Open your browser and navigate to `http://localhost:3000`
2. Click on "Navigation" in the navigation bar
3. The map will load with offline tiles

### GPS Features
- **Real-time Position**: Blue dot shows current rover position
- **Heading Indicator**: White line on the blue dot shows rover direction
- **Accuracy Circle**: Blue circle shows GPS accuracy radius
- **Status Display**: Shows GPS signal quality and satellite count

### Navigation Features
- **Distance Calculation**: Shows distance to selected waypoint
- **Bearing Calculation**: Shows direction to selected waypoint
- **Closest Waypoint**: Automatically identifies nearest waypoint

### Waypoint Management
- **Add Waypoints**: Click anywhere on the map to add a waypoint
- **Edit Waypoints**: Click on a waypoint to select it (turns red)
- **Export/Import**: Download/upload waypoints as JSON files
- **Data Management**: Use the sidebar to manage waypoints

## Map Features
- ✅ **Offline Operation** - No internet required
- ✅ **Multiple Styles** - OSM, Satellite, Terrain views
- ✅ **GPS Tracking** - Real-time position on offline maps
- ✅ **Waypoint Management** - Mark and navigate to points
- ✅ **Route Planning** - Plan routes on offline maps
- ✅ **High Detail** - Configurable zoom levels (10-18)

## File Sizes and Times
| Area Type | Zoom Levels | Size | Time |
|-----------|-------------|------|------|
| Small campus | 12-16 | ~500MB | 10-20 min |
| Medium city | 10-16 | ~2GB | 20-40 min |
| Large area | 10-18 | ~5GB | 30-60 min |

## File Structure

```
services/gps/
├── gps_service.py              # Main GPS service (ROS integration)
├── gps_api.py                  # Flask API
├── docker-compose.tileserver.yml # TileServer setup
├── README.md                   # This documentation
└── download-scripts/           # Map tile downloaders
    ├── download-mcgill-basic.sh      # Basic McGill campus tiles
    ├── download-mcgill-complete.sh   # Complete McGill coverage
    ├── download-drumheller-tiles.sh  # Basic Drumheller tiles
    ├── download-drumheller-complete.sh # Complete Drumheller coverage
    └── download-custom-area.sh       # Custom area downloader
```

## Dependencies

### System Dependencies
- **Docker** - For TileServer-GL
- **ROS2** - For GPS data collection
- **Flask** - Web API framework

### Python Dependencies
- **flask** - Web framework
- **flask-cors** - Cross-origin support
- **rclpy** - ROS2 Python client
- **sensor_msgs** - ROS2 sensor messages

## Troubleshooting

### Common Issues
1. **No GPS data** - Check ROS topics and GPS device
2. **Map tiles not loading** - Verify TileServer is running
3. **High memory usage** - Reduce tile zoom levels
4. **Slow tile downloads** - Use smaller bounding boxes

### Debug Commands
```bash
# Check GPS service status
curl http://localhost:5001/api/health

# Check GPS data stream
curl http://localhost:5001/api/gps/stream

# Check TileServer status
curl http://localhost:8080/

# Check ROS GPS topics
ros2 topic list | grep gps
ros2 topic echo /gps/fix
```

### Performance Issues
1. Use basic download scripts instead of complete ones
2. Reduce map zoom levels in custom area scripts
3. Limit the map area bounds
4. Use lower quality tile formats

## Development

### Adding New Map Styles
1. Create a new style file in `styles/`
2. Update TileServer configuration in `service_config.yml`
3. Add the style to the UI map style selector

### Extending GPS Features
1. Modify `gps_service.py` to add new GPS data fields
2. Update the API endpoints in `gps_api.py`
3. Extend the React hooks in `useGPSData.ts`
4. Update the UI components to display new data

### Custom Waypoint Features
1. Add new waypoint properties in the TypeScript interfaces
2. Extend the waypoint management components
3. Update the export/import functionality
4. Add new navigation features

## Integration Notes

This GPS service is designed to work seamlessly with:
- **ROS system** in the `services/ros/` folder
- **React UI** in the `robot-controller-ui/` folder
- **Service manager** in the parent `services/` folder

The service provides both real-time GPS data and offline mapping capabilities for the teleop system.

## License

This offline mapping implementation uses:
- MapLibre GL JS (Apache 2.0)
- TileServer-GL (BSD 3-Clause)
- OpenStreetMap (ODbL)
- Flask (BSD 3-Clause) 