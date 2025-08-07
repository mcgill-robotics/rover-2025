# Offline Mapping for Rover 2025

This document describes the offline mapping solution implemented for the rover project, which allows GPS functionality without requiring internet connectivity.

## Overview

The offline mapping system consists of:

1. **MapLibre GL JS** - Open-source mapping library for the frontend
2. **TileServer-GL** - Serves offline map tiles
3. **OpenStreetMap (OSM)** - Open-source map data
4. **GPS Integration** - ROS-based GPS service with REST API

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Rover UI      │    │   GPS Service   │    │  TileServer-GL  │
│  (React/Next.js)│◄──►│   (ROS/Flask)   │    │   (Docker)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  MapLibre GL JS │    │   ROS Topics    │    │   Offline Tiles │
│   (Frontend)    │    │  (/gps/fix,     │    │   (.mbtiles)    │
└─────────────────┘    │   /imu/data)    │    └─────────────────┘
                       └─────────────────┘
```

## Features

- ✅ **Offline Map Tiles** - No internet required for map display
- ✅ **Real-time GPS Tracking** - Live rover position and heading
- ✅ **Waypoint Management** - Add, edit, delete, and navigate to waypoints
- ✅ **Multiple Map Styles** - Street, satellite, and terrain views
- ✅ **Distance & Bearing Calculation** - Navigation assistance
- ✅ **Export/Import Waypoints** - JSON format for data portability
- ✅ **GPS Status Monitoring** - Signal quality and satellite count

## Setup Instructions

### 1. Install Dependencies

```bash
# Navigate to the teleop directory
cd rover-2025/teleop

# Install Node.js dependencies for the UI
cd robot-controller-ui
npm install

# Install Python dependencies for GPS service
cd ../services/gps
pip install flask flask-cors rclpy sensor_msgs geometry_msgs
```

### 2. Download Offline Map Tiles

```bash
# Run the tile download script
cd rover-2025/teleop
./download-map-tiles.sh
```

This script will:
- Create necessary directories (`mbtiles`, `styles`, `fonts`)
- Download OpenStreetMap tiles for the specified area
- Generate map style configurations

### 3. Start TileServer-GL

```bash
# Start the offline tile server
docker-compose -f docker-compose.tileserver.yml up -d
```

The tile server will be available at `http://localhost:8080`

### 4. Start GPS Service

```bash
# Start the GPS service (requires ROS environment)
cd rover-2025/teleop/services/gps
python3 gps_api.py
```

The GPS API will be available at `http://localhost:5001`

### 5. Start the Rover UI

```bash
# Start the React development server
cd rover-2025/teleop/robot-controller-ui
npm run dev
```

The UI will be available at `http://localhost:3000`

## Configuration

### Map Area Configuration

Edit `download-map-tiles.sh` to change the map area:

```bash
# Current configuration (McGill University area)
BOUNDS="45.5048,-73.5772,45.5148,-73.5672"
MIN_ZOOM=12
MAX_ZOOM=18
```

### GPS Topics

Edit `services/gps/gps_service.py` to match your ROS topic names:

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

### TileServer Configuration

Edit `tileserver-config.json` to customize the tile server:

```json
{
  "options": {
    "paths": {
      "root": "/usr/src/app",
      "fonts": "fonts",
      "styles": "styles",
      "mbtiles": "mbtiles"
    }
  }
}
```

## Usage

### Accessing the Mapping Interface

1. Open your browser and navigate to `http://localhost:3000`
2. Click on "Mapping" in the navigation bar
3. The map will load with offline tiles

### Adding Waypoints

1. Click anywhere on the map to add a waypoint
2. Waypoints appear as green dots on the map
3. Click on a waypoint to select it (turns red)
4. Use the sidebar to manage waypoints

### GPS Features

- **Real-time Position**: Blue dot shows current rover position
- **Heading Indicator**: White line on the blue dot shows rover direction
- **Accuracy Circle**: Blue circle shows GPS accuracy radius
- **Status Display**: Shows GPS signal quality and satellite count

### Navigation

- **Distance Calculation**: Shows distance to selected waypoint
- **Bearing Calculation**: Shows direction to selected waypoint
- **Closest Waypoint**: Automatically identifies nearest waypoint

### Data Management

- **Export Waypoints**: Download waypoints as JSON file
- **Import Waypoints**: Upload previously saved waypoint files
- **Edit Waypoints**: Click the edit icon to modify waypoint details

## API Endpoints

### GPS Service API

- `GET /api/gps/data` - Get current GPS data
- `GET /api/gps/status` - Get GPS status information
- `GET /api/gps/stream` - Stream GPS data (Server-Sent Events)
- `GET /api/health` - Health check
- `GET /api/config` - Get GPS configuration

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

## Troubleshooting

### Map Not Loading

1. Check if TileServer-GL is running: `docker ps`
2. Verify tileserver is accessible: `curl http://localhost:8080`
3. Check browser console for errors
4. Ensure map tiles were downloaded correctly

### GPS Data Not Updating

1. Check if GPS service is running: `ps aux | grep gps_api`
2. Verify ROS topics are publishing: `ros2 topic list`
3. Check GPS API health: `curl http://localhost:5001/api/health`
4. Review GPS service logs for errors

### Performance Issues

1. Reduce map zoom levels in `download-map-tiles.sh`
2. Limit the map area bounds
3. Use lower quality tile formats
4. Consider using vector tiles instead of raster tiles

## Development

### Adding New Map Styles

1. Create a new style file in `styles/`
2. Update `tileserver-config.json`
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

## File Structure

```
rover-2025/teleop/
├── robot-controller-ui/
│   ├── src/
│   │   ├── app/mapping/
│   │   │   └── page.tsx
│   │   ├── components/sections/mapping/
│   │   │   ├── MappingSection.tsx
│   │   │   ├── OfflineMap.tsx
│   │   │   ├── GPSDisplay.tsx
│   │   │   └── WaypointManager.tsx
│   │   └── hooks/
│   │       └── useGPSData.ts
│   └── package.json
├── services/gps/
│   ├── gps_service.py
│   └── gps_api.py
├── docker-compose.tileserver.yml
├── tileserver-config.json
├── download-map-tiles.sh
└── OFFLINE_MAPPING_README.md
```

## Contributing

When contributing to the offline mapping system:

1. Follow the existing code style and patterns
2. Add appropriate TypeScript types for new features
3. Update this documentation for any new features
4. Test with both online and offline scenarios
5. Ensure GPS integration works with real ROS data

## License

This offline mapping implementation uses:
- MapLibre GL JS (Apache 2.0)
- TileServer-GL (BSD 3-Clause)
- OpenStreetMap (ODbL)
- Flask (BSD 3-Clause) 