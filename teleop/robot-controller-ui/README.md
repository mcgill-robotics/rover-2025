# Robot Controller UI

A Next.js-based web interface for controlling and monitoring the Mars Rover's teleoperation systems. This application provides real-time data visualization and control interfaces for the rover's drive system, arm control, and camera feeds.

## 🏗️ Project Structure

```
teleop/robot-controller-ui/
├── src/
│   ├── app/                    # Next.js App Router pages
│   │   ├── drive/             # Drive system control page
│   │   ├── arm/               # Arm control page
│   │   ├── cameras/           # Camera feeds page
│   │   └── layout.tsx         # Root layout component
│   ├── components/            # Reusable UI components
│   │   ├── sections/          # Page-specific component sections
│   │   │   ├── drive/         # Drive system components
│   │   │   │   ├── mobility/  # Drive mobility controls
│   │   │   │   │   ├── info/  # Drive information displays
│   │   │   │   │   │   ├── MotorPanel/      # Motor diagnostic panels
│   │   │   │   │   │   ├── Speedometer/    # Speed visualization
│   │   │   │   │   │   └── ChartPanel/     # Data charts
│   │   │   │   │   └── control/            # Drive control interfaces
│   │   │   ├── arm/           # Arm control components
│   │   │   └── cameras/       # Camera display components
│   │   ├── ui/                # Generic UI components
│   │   └── layout/            # Layout components (navbar, sidebar)
│   ├── hooks/                 # Custom React hooks
│   │   └── useDriveData.ts    # Legacy drive data hook (replaced by Zustand)
│   ├── store/                 # Global state management (Zustand)
│   │   ├── types.ts           # TypeScript type definitions
│   │   ├── rosStore.ts        # Main ROS data store
│   │   └── index.ts           # Store exports
│   ├── services/              # External service integrations
│   └── styles/                # Global styles and Tailwind config
├── public/                    # Static assets
├── package.json              # Dependencies and scripts
└── README.md                 # This file
```

## 🚀 Getting Started

### Prerequisites

- Node.js 18+ and npm
- ROS2 environment (for backend services)
- Python 3.8+ (for ROS services)

### Installation

1. **Install frontend dependencies:**
   ```bash
   cd teleop/robot-controller-ui
   npm install
   ```

2. **Install ROS service dependencies:**
   ```bash
   cd teleop/services
   pip install -r requirements.txt
   ```

### Running the Application

#### 1. Start ROS Services

First, ensure your ROS2 environment is sourced and start the required ROS nodes:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash  # or your ROS2 installation
source ~/ros2_ws/install/setup.bash

# Start drive firmware node (publishes motor data)
ros2 run control drive_firmware_node

# In another terminal, start the ROS Manager (bridges ROS to web)
cd teleop/services/ros
python ros_manager.py
```

The ROS Manager will start a web server on port 8082 that provides:
- REST API endpoints for drive data
- WebSocket connection for real-time updates
- Bridge between ROS2 topics/services and web interface

#### 2. Start Frontend Development Server

```bash
cd teleop/robot-controller-ui
npm run dev
```

The web interface will be available at `http://localhost:3000`

#### 3. Production Build

```bash
npm run build
npm start
```

## 🏛️ Architecture Overview

### Frontend Architecture

The application uses a modern React architecture with:

- **Next.js 15** - React framework with App Router
- **TypeScript** - Type safety and better developer experience
- **Tailwind CSS** - Utility-first CSS framework
- **Zustand** - Lightweight state management for ROS data
- **Recharts** - Data visualization library

### State Management (Zustand Store)

The application uses Zustand for global state management, specifically for ROS data:

```typescript
// Access drive diagnostics
const diagnostics = useDriveDiagnostics();

// Access connection status
const isConnected = useIsROSConnected();

// Manual connection control
const { connect, disconnect, reconnect } = useAutoConnect();
```

**Store Structure:**
- `drive`: Motor diagnostics, speeds, status, and connection info
- `arm`: Arm joint data and end-effector position (future)
- `connection`: WebSocket connection state and error handling

### ROS Integration

The system bridges ROS2 and web technologies through:

1. **ROS Manager** (`teleop/services/ros/ros_manager.py`)
   - Manages ROS2 node lifecycle
   - Provides REST API and WebSocket endpoints
   - Handles data serialization and real-time streaming

2. **Drive Data Subscriber** (`teleop/services/ros/drive/subscriber/drive_data_subscriber.py`)
   - Subscribes to drive system topics
   - Collects motor diagnostics, speeds, and status
   - Provides service client for motor status requests

3. **Frontend Store** (`src/store/rosStore.ts`)
   - Manages WebSocket connection to ROS Manager
   - Provides reactive state updates
   - Handles connection recovery and error states

## 📊 Data Flow

```
ROS2 Nodes → ROS Manager → WebSocket → Frontend Store → React Components
     ↓              ↓           ↓            ↓              ↓
Drive Firmware → Subscriber → REST API → HTTP Requests → UI Updates
```

### Real-time Data Updates

1. **ROS Topics**: Drive firmware publishes to ROS topics
2. **ROS Subscriber**: Collects and processes ROS messages
3. **WebSocket Streaming**: Real-time data pushed to frontend
4. **Zustand Store**: Updates global state reactively
5. **React Components**: Automatically re-render with new data

## 🔧 Key Components

### Drive System

**DriveInfo Component** (`src/components/sections/drive/mobility/info/DriveInfo.tsx`)
- Main drive system dashboard
- Real-time motor diagnostics display
- Connection status indicator
- Automatic ROS connection management

**Motor Diagnostics** (`src/components/sections/drive/mobility/info/MotorPanel/`)
- Individual motor status panels
- Voltage, current, temperature monitoring
- Alert system for motor issues
- Tabbed interface with chart view

**Speedometer Cluster** (`src/components/sections/drive/mobility/info/Speedometer/`)
- Visual speed representation
- Individual wheel speed displays
- Main rover speed indicator

### State Management

**ROS Store** (`src/store/rosStore.ts`)
- Centralized ROS data management
- WebSocket connection handling
- Automatic reconnection logic
- Type-safe data access

## 🌐 API Endpoints

The ROS Manager provides the following endpoints:

### REST API (Port 8082)

- `GET /api/drive/diagnostics` - Current motor diagnostics
- `GET /api/drive/speeds` - Current motor speeds
- `GET /api/drive/status` - Motor connection status
- `GET /api/drive/summary` - Complete drive system summary
- `GET /api/health` - Service health check

### WebSocket (Port 8082)

- `ws://localhost:8082/ws` - Real-time data streaming
- Automatic subscription to drive topics
- Ping/pong for connection health
- Error handling and recovery

## 🔌 ROS Topics & Services

### Subscribed Topics

- `drive_motors_info` (DriveMotorDiagnostic) - Motor voltage, current, temperature
- `drive_speeds_info` (Float32MultiArray) - Motor speeds [RF, RB, LB, LF]

### Service Clients

- `drive_motors_status` (DriveMotorStatus) - Motor connection status

## 🚨 Troubleshooting

### Common Issues

1. **"ROS Disconnected" Status**
   - Ensure ROS2 environment is sourced
   - Check if drive_firmware_node is running
   - Verify ROS Manager is started on port 8082

2. **No Data Updates**
   - Check WebSocket connection in browser dev tools
   - Verify ROS topics are publishing: `ros2 topic list`
   - Check ROS Manager logs for errors

3. **Build Errors**
   - Clear Next.js cache: `rm -rf .next`
   - Reinstall dependencies: `rm -rf node_modules && npm install`
   - Check TypeScript errors: `npm run build`

### Development Tips

1. **Enable Zustand DevTools**
   ```typescript
   // In rosStore.ts, devtools are already enabled
   // Use Redux DevTools browser extension to inspect state
   ```

2. **Debug WebSocket Connection**
   ```javascript
   // In browser console
   const ws = new WebSocket('ws://localhost:8082/ws');
   ws.onmessage = (event) => console.log(JSON.parse(event.data));
   ```

3. **Monitor ROS Topics**
   ```bash
   # Check if topics are publishing
   ros2 topic echo /drive_motors_info
   ros2 topic echo /drive_speeds_info
   
   # Check service availability
   ros2 service list | grep drive
   ```

## 🔮 Future Enhancements

### Planned Features

1. **Arm Control Integration**
   - Joint position control
   - End-effector visualization
   - Inverse kinematics interface

2. **Camera System**
   - Multiple camera feeds
   - WebRTC streaming
   - Pan/tilt control

3. **Enhanced Diagnostics**
   - Historical data logging
   - Performance analytics
   - Predictive maintenance alerts

4. **Mobile Responsiveness**
   - Touch-friendly controls
   - Responsive layouts
   - Progressive Web App features

### Architecture Improvements

1. **Service Discovery**
   - Automatic ROS node detection
   - Dynamic topic subscription
   - Health monitoring dashboard

2. **Data Persistence**
   - Local storage for settings
   - Session replay capability
   - Offline mode support

3. **Security**
   - Authentication system
   - Role-based access control
   - Secure WebSocket connections

## 📝 Contributing

1. Follow TypeScript best practices
2. Use Tailwind CSS for styling
3. Maintain component modularity
4. Add proper error handling
5. Update documentation for new features

## 📄 License

This project is part of the McGill Robotics Mars Rover team codebase.
