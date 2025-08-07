# ROS Drive Data Service

This service provides drive data integration between ROS2 topics and the frontend web interface for motor control and drive diagnostics.

## Overview

The drive service consists of:
- **Drive Data Subscriber**: ROS2 node that subscribes to motor diagnostics and drive data
- **Web API Endpoints**: REST endpoints for accessing drive data
- **WebSocket Broadcasting**: Real-time data streaming to the frontend
- **Frontend Components**: React components for displaying drive status and motor diagnostics

## ROS Topics

The service subscribes to the following ROS2 topics:

### Drive Motor Diagnostics
- **Topic**: `/drive/motor_diagnostics`
- **Message Type**: `msg_interface/msg/DriveMotorDiagnostic`
- **Contains**: Motor voltage, current, temperature, status

### Drive Commands
- **Topic**: `/drive/cmd_vel`
- **Message Type**: `geometry_msgs/msg/Twist`
- **Contains**: Linear and angular velocity commands

### Drive Status
- **Topic**: `/drive/status`
- **Message Type**: `msg_interface/msg/DriveMotorStatus`
- **Contains**: Overall drive system status

## API Endpoints

### Drive Diagnostics
- **GET** `/api/drive/diagnostics`
- **Response**: Current motor diagnostics with timestamps

### Drive Status
- **GET** `/api/drive/status`
- **Response**: Overall drive system status

### Drive Commands
- **POST** `/api/drive/command`
- **Body**: `{"linear_x": 0.5, "angular_z": 0.0}`
- **Response**: Command acknowledgment

## Frontend Components

### Drive Control Component
Located at: `src/components/sections/drive/mobility/control/DriveControl.tsx`

Features:
- Real-time motor diagnostics display
- Drive command interface
- Speed control sliders
- Safety limits and warnings
- Motor status indicators

### Drive Info Component
Located at: `src/components/sections/drive/mobility/info/DriveInfo.tsx`

Features:
- Motor voltage and current monitoring
- Temperature tracking
- System status overview
- Performance metrics

## Setup and Installation

### Backend Dependencies
The drive service uses the existing ROS bridge infrastructure. No additional Python dependencies are required.

### Frontend Dependencies
The required npm packages are already included in the main package.json.

## Testing

### Mock Drive Publisher
A mock drive publisher is provided for testing:

```bash
cd rover-2025/teleop/services/ros/tests
python3 test_mock_drive_publisher.py
```

This will publish sample drive data to the required topics.

### Running the Service

1. Start the ROS manager:
```bash
cd rover-2025/teleop/services/ros
python3 ros_manager.py
```

2. Start the mock drive publisher (in another terminal):
```bash
cd rover-2025/teleop/services/ros/tests
python3 test_mock_drive_publisher.py
```

3. Start the frontend:
```bash
cd rover-2025/teleop/robot-controller-ui
npm run dev
```

## Data Flow

```
Rover Motors → ROS Topics → Drive Subscriber → ROS Manager → Frontend
     ↓              ↓              ↓              ↓           ↓
  Motor Data   /drive/motor   Process Data   Web API    React UI
  Hardware     /drive/status  & Store       Endpoints   Components
```

## Troubleshooting

### Common Issues

1. **No drive data appearing**:
   - Check if ROS topics are being published: `ros2 topic list`
   - Verify topic names match expected format
   - Check ROS manager logs

2. **Frontend not receiving data**:
   - Verify WebSocket connection
   - Check browser console for errors
   - Ensure ROS manager is running

3. **Motor commands not working**:
   - Check if drive command topic is subscribed
   - Verify message format
   - Check motor hardware connections

## Development

### Adding New Drive Features

1. **New ROS Topics**: Add subscribers in `drive_data_subscriber.py`
2. **New API Endpoints**: Add to ROS manager web API
3. **New Frontend Components**: Create React components in drive section
4. **Testing**: Add test cases in `services/ros/tests/`

### Message Types

The service uses custom ROS messages defined in `msg_interface`:
- `DriveMotorDiagnostic.msg` - Motor diagnostics data
- `DriveMotorStatus.msg` - Overall drive status
- `GamePadInput.msg` - Gamepad input data 