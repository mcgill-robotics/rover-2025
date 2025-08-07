# Robot Controller UI

This folder contains the React/Next.js frontend for the Rover 2025 teleop system.

## Overview

The Robot Controller UI is the main user interface for controlling the rover. It provides:
- **Drive Control**: Real-time drive control with gamepad support
- **Arm Control**: Robotic arm manipulation interface
- **Offline Mapping**: GPS-based mapping with offline map tiles
- **System Status**: Real-time monitoring of all services

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   React UI      │    │   Backend       │    │   Rover         │
│   (Next.js)     │◄──►│   Services      │◄──►│   Hardware      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  User Interface │    │  Service APIs   │    │  ROS Topics     │
│  (Port 3000)    │    │  (Ports 8082+)  │    │  (Drive/Arm)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Pages (`src/app/`)
- **`page.tsx`** - Main dashboard
- **`drive/page.tsx`** - Drive control interface
- **`arm/page.tsx`** - Arm control interface
- **`mapping/page.tsx`** - Offline mapping interface
- **`status/page.tsx`** - System status monitoring

### Components (`src/components/`)
- **`sections/drive/`** - Drive control components
- **`sections/arm/`** - Arm control components
- **`sections/mapping/`** - Mapping components
- **`layout/navbar/`** - Navigation components
- **`ui/`** - Reusable UI components

### Hooks (`src/hooks/`)
- **`useDriveData.ts`** - Drive data management
- **`useGPSData.ts`** - GPS data streaming
- **`useMultiCameraStream.ts`** - Camera stream management
- **`useBandwidthStats.ts`** - Network statistics

### Store (`src/store/`)
- **`rosStore.ts`** - ROS data state management
- **`types.ts`** - TypeScript type definitions

## Integration with Backend Services

### ROS Manager (Port 8082)
- Real-time drive control commands
- Arm manipulation commands
- Motor status and diagnostics

### GPS Service (Port 5001)
- GPS coordinates for mapping
- IMU data for orientation
- Server-Sent Events (SSE) streaming

### Camera Service (Port 8001)
- Multi-camera video streams
- ArUco marker detection
- WebSocket video streaming

### TileServer (Port 8080)
- Offline map tiles
- OpenStreetMap data
- Vector/raster tile serving

## Development

### Prerequisites
```bash
# Install Node.js dependencies
npm install

# Install additional dependencies for mapping
npm install maplibre-gl react-maplibre-gl
```

### Running the UI
```bash
# Development mode
npm run dev

# Production build
npm run build
npm start
```

### Environment Variables
```bash
# Backend service URLs
NEXT_PUBLIC_ROS_API_URL=http://localhost:8082
NEXT_PUBLIC_GPS_API_URL=http://localhost:5001
NEXT_PUBLIC_CAMERA_API_URL=http://localhost:8001
NEXT_PUBLIC_TILESERVER_URL=http://localhost:8080
```

## Features

### Drive Control
- ✅ Real-time gamepad support
- ✅ Motor diagnostics display
- ✅ Speed and direction control
- ✅ Emergency stop functionality

### Arm Control
- ✅ 6-DOF arm manipulation
- ✅ Inverse kinematics
- ✅ Joint limit monitoring
- ✅ Trajectory planning

### Offline Mapping
- ✅ GPS coordinate display
- ✅ Offline map tiles
- ✅ Waypoint management
- ✅ Route planning

### System Monitoring
- ✅ Service health status
- ✅ Network bandwidth stats
- ✅ Camera stream monitoring
- ✅ Real-time diagnostics

## File Structure

```
robot-controller-ui/
├── src/
│   ├── app/                    # Next.js pages
│   │   ├── drive/             # Drive control page
│   │   ├── arm/               # Arm control page
│   │   ├── mapping/           # Mapping page
│   │   └── status/            # Status page
│   ├── components/            # React components
│   │   ├── sections/          # Page-specific sections
│   │   ├── layout/            # Layout components
│   │   └── ui/                # Reusable UI components
│   ├── hooks/                 # Custom React hooks
│   ├── store/                 # State management
│   └── config/                # Configuration files
├── public/                    # Static assets
│   ├── gamepads/             # Gamepad icons
│   └── mcgillRobotics.svg    # Logo
├── package.json              # Dependencies
└── next.config.ts            # Next.js configuration
```

## Integration Notes

This UI is designed to work seamlessly with the backend services in the `services/` folder. It communicates via:
- **REST APIs** for control commands
- **WebSockets** for real-time data
- **Server-Sent Events** for GPS streaming
- **HTTP** for map tile requests

The UI automatically adapts to available services and provides fallbacks when services are unavailable.

---

**Note**: This is the frontend component of the teleop system. For backend services, see the `services/` folder documentation.
