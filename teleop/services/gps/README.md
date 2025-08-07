# GPS Services

This folder contains the GPS and offline mapping services for the Rover 2025 teleop system.

## Overview

The GPS Services provide GPS data collection, offline mapping capabilities, and map tile serving for the teleop system. This enables GPS-based navigation and mapping without requiring internet connectivity.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   GPS Device    │    │   GPS Service   │    │   React UI      │
│   (Hardware)    │───►│   (Flask API)   │───►│   (Frontend)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  GPS/IMU Data   │    │  SSE Streaming  │    │  Mapping Display│
│  (ROS Topics)   │    │  (Port 5001)    │    │  (Port 3000)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Core Files
- **`gps_service.py`** - Main GPS data service
- **`gps_api.py`** - Flask API for GPS data
- **`download-map-tiles.sh`** - Map tile downloader
- **`docker-compose.tileserver.yml`** - TileServer-GL setup
- **`OFFLINE_MAPPING_README.md`** - Detailed mapping documentation

### Configuration
- **`service_config.yml`** - Service configuration (in parent `services/` folder)
- **TileServer-GL** - Offline map tile serving

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
```

### TileServer Configuration
```yaml
tileserver:
  port: 8080
  enabled: true
  docker_compose_file: "gps/docker-compose.tileserver.yml"
  config:
    options:
      paths:
        root: "/usr/src/app"
        mbtiles: "mbtiles"
    styles:
      osm-bright:
        style: "osm-bright/style.json"
    data:
      v3:
        mbtiles: "v3.mbtiles"
```

## API Endpoints

### REST Endpoints
- **`GET /api/health`** - Service health check
- **`GET /api/gps/current`** - Current GPS coordinates
- **`GET /api/gps/history`** - GPS history data
- **`GET /api/gps/status`** - GPS device status

### Server-Sent Events
- **`/api/gps/stream`** - Real-time GPS data stream
- **`/api/imu/stream`** - Real-time IMU data stream

## Usage

### Starting the Service
```bash
# Via service manager (recommended)
cd services
./start_services.sh --mode mapping

# Direct startup
python3 gps/gps_service.py
```

### Downloading Map Tiles
```bash
# Download tiles for specific area
cd services/gps
./download-map-tiles.sh --bounds "45.5,-73.6,45.6,-73.5" --zoom "10-15"

# Download tiles for Montreal area
./download-map-tiles.sh --city "Montreal" --zoom "10-15"
```

### Starting TileServer
```bash
# Start TileServer-GL
cd services/gps
docker-compose -f docker-compose.tileserver.yml up -d

# Check TileServer status
curl http://localhost:8080/
```

### Integration with UI
The React UI automatically connects to the GPS service and displays:
- Real-time GPS coordinates
- Offline map tiles
- Waypoint management
- Route planning

## File Structure

```
services/gps/
├── gps_service.py              # Main GPS service
├── gps_api.py                  # Flask API
├── download-map-tiles.sh       # Map tile downloader
├── docker-compose.tileserver.yml # TileServer setup
├── OFFLINE_MAPPING_README.md   # Detailed mapping docs
└── README.md                   # This documentation
```

## Dependencies

### System Dependencies
- **Docker** - For TileServer-GL
- **ROS2** - For GPS data collection
- **Flask** - Web API framework
- **GPSD** - GPS device interface

### Python Dependencies
- **flask** - Web framework
- **flask-cors** - Cross-origin support
- **rclpy** - ROS2 Python client
- **sensor_msgs** - ROS2 sensor messages

## Map Data

### Supported Map Styles
- **OSM Bright** - Street map with buildings
- **Satellite** - Aerial imagery
- **Terrain** - Topographic maps
- **Custom** - User-defined styles

### Tile Formats
- **MBTiles** - SQLite-based tile storage
- **PNG/JPEG** - Raster tile formats
- **Vector Tiles** - PBF format support

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

## Integration Notes

This GPS service is designed to work seamlessly with:
- **ROS system** in the `services/ros/` folder
- **React UI** in the `robot-controller-ui/` folder
- **Service manager** in the parent `services/` folder

The service provides both real-time GPS data and offline mapping capabilities for the teleop system.

---

**Note**: This is the GPS and mapping component of the teleop system. For detailed mapping setup, see `OFFLINE_MAPPING_README.md`. 