# GPS Service

This service provides GPS and IMU data integration between ROS2 topics and the frontend web interface.

## Overview

The GPS service consists of:
- **GPS Data Subscriber**: ROS2 node that subscribes to GPS coordinates and IMU data
- **Web API Endpoints**: REST endpoints for accessing GPS and IMU data
- **WebSocket Broadcasting**: Real-time data streaming to the frontend
- **Frontend Components**: React components for displaying GPS data and maps

## ROS Topics

The service subscribes to the following ROS2 topics:

### GPS Coordinates
- **Topic**: `/gps_coordinates`
- **Message Type**: `std_msgs/msg/Float64MultiArray`
- **Format**: `[latitude, longitude]`
- **Example**: `[45.5049216, -73.56316]`

### IMU Data
- **Topic**: `/imu_data`
- **Message Type**: `sensor_msgs/msg/Imu`
- **Contains**: Orientation, angular velocity, linear acceleration

## API Endpoints

### GPS Coordinates
- **GET** `/api/gps/coordinates`
- **Response**: Current GPS coordinates with timestamp

### IMU Data
- **GET** `/api/gps/imu`
- **Response**: Current IMU data with timestamp

### GPS Summary
- **GET** `/api/gps/summary`
- **Response**: Summary of GPS and IMU data with connection status

## Frontend Components

### GPS Component
Located at: `src/components/sections/drive/mobility/navigation/GPS.tsx`

Features:
- Real-time GPS coordinate display
- Interactive map with Leaflet
- Waypoint management
- Current location tracking
- Path visualization

### CurrentLocation Component
Located at: `src/components/sections/drive/mobility/navigation/CurrentLocation.tsx`

Features:
- Editable coordinate inputs
- Real-time updates from ROS data

## Setup and Installation

### Backend Dependencies
The GPS service uses the existing ROS bridge infrastructure. No additional Python dependencies are required.

### Frontend Dependencies
Install the required npm packages:

```bash
cd rover-2025/teleop/robot-controller-ui
npm install leaflet react-leaflet roslib @types/leaflet
```

## Testing

### Mock GPS Publisher
A mock GPS publisher is provided for testing:

```bash
cd rover-2025/teleop/services/ros/gps
python3 test_mock_gps_publisher.py
```

This will publish sample GPS and IMU data to the required topics.

### Running the Service

1. Start the ROS manager:
```bash
cd rover-2025/teleop/services/ros
python3 ros_manager.py
```

2. Start the mock GPS publisher (in another terminal):
```bash
cd rover-2025/teleop/services/ros/gps
python3 test_mock_gps_publisher.py
```

3. Start the frontend:
```bash
cd rover-2025/teleop/robot-controller-ui
npm run dev
```

4. Navigate to the GPS panel in the frontend to see the real-time data.

## Integration

The GPS service is automatically integrated into the ROS manager and will:
- Subscribe to GPS and IMU topics when initialized
- Broadcast data updates via WebSocket
- Provide REST API endpoints for data access
- Update the frontend in real-time

## Configuration

### Topic Names
You can modify the topic names in `gps_data_subscriber.py`:
- GPS topic: `self.gps_subscriber = self.create_subscription(Float64MultiArray, '/gps_coordinates', ...)`
- IMU topic: `self.imu_subscriber = self.create_subscription(Imu, '/imu_data', ...)`

### Update Frequency
The service processes data as it arrives from ROS topics. The mock publisher sends data every 1 second.

### Connection Timeout
GPS and IMU data are considered stale after 5 seconds without updates. This can be modified in the `get_connection_status()` method. 