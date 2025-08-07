# Camera Services

This folder contains the camera backend service for the Rover 2025 teleop system.

## Overview

The Camera Service provides multi-camera support with RTP streaming, ArUco marker detection, and real-time video processing. It acts as the backend for camera data that the React UI consumes.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Jetson/Pi     │    │   Camera        │    │   React UI      │
│   Cameras       │───►│   Service       │───►│   (Frontend)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  RTP Streams    │    │  WebSocket API  │    │  Video Display  │
│  (H.264)        │    │  (Port 8001)    │    │  (Port 3000)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Key Components

### Core Files
- **`camera_service.py`** - Main FastAPI service (renamed from `central_backend.py`)
- **`gstreamer_reader.py`** - GStreamer video processing and RTP handling
- **`aruco_detector.py`** - ArUco marker detection and processing

### Configuration
- **`service_config.yml`** - Service configuration (in parent `services/` folder)
- **Dynamic Port Assignment** - No fixed ports, Jetson server chooses any available port

## Features

### Multi-Camera Support
- ✅ **Dynamic Discovery** - Automatically finds cameras on any port
- ✅ **RTP Streaming** - H.264 video streams from Jetson devices
- ✅ **Real-time Processing** - Low-latency video processing
- ✅ **Auto-Connect** - Automatically connects to discovered cameras

### Video Processing
- ✅ **GStreamer Pipeline** - Professional video processing
- ✅ **H.264 Decoding** - Hardware-accelerated video decoding
- ✅ **Frame Buffering** - Optimized frame handling
- ✅ **Drop Frame Support** - Maintains real-time performance

### ArUco Detection
- ✅ **Marker Detection** - Real-time ArUco marker identification
- ✅ **Pose Estimation** - 3D position and orientation calculation
- ✅ **Visual Overlay** - Draws markers and IDs on video
- ✅ **Configurable** - Adjustable detection parameters

### Web API
- ✅ **FastAPI Backend** - Modern, fast web framework
- ✅ **WebSocket Support** - Real-time video streaming
- ✅ **REST Endpoints** - Camera management and status
- ✅ **Health Monitoring** - Service health checks

## Integration with Teleop System

### Jetson/Vision Side (`rover-2025/vision/`)
- **`jetson_server.py`** - Multi-camera capture and RTP streaming
- **`run_jetson.sh`** - Jetson startup script
- **`config.yml`** - Jetson-specific configuration

### React UI (`robot-controller-ui/`)
- **`useMultiCameraStream.ts`** - Camera stream management hook
- **`sections/drive/camera/`** - Camera display components
- **WebSocket connections** - Real-time video streaming

### Service Manager (`services/`)
- **`service_manager.py`** - Manages camera service lifecycle
- **`service_config.yml`** - Central configuration
- **Health monitoring** - Service status tracking

## Configuration

### Service Configuration (`service_config.yml`)
```yaml
camera_service:
  port: 8001
  enabled: false  # Enable when needed
  config:
    host: "0.0.0.0"
    inactive_timeout: 30.0
    frame_buffer_size: 10
    camera_discovery:
      enabled: true
      scan_interval: 5.0
      auto_connect: true
    gstreamer_config:
      rtp_caps: "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96"
      pipeline_elements: [...]
    aruco_config:
      dictionary: "DICT_4X4_100"
      marker_border_color: [0, 255, 0]
    jpeg_config:
      quality: 85
      optimize: true
```

### Dynamic Port Assignment
- **No Fixed Ports** - Jetson server chooses any available port
- **Automatic Discovery** - Backend scans for cameras on any port
- **Scalable** - Can handle unlimited cameras without configuration

## API Endpoints

### REST Endpoints
- **`GET /api/health`** - Service health check
- **`GET /api/cameras`** - List connected cameras
- **`GET /api/cameras/{camera_id}`** - Camera details
- **`POST /api/cameras/{camera_id}/restart`** - Restart camera

### WebSocket Endpoints
- **`/ws/camera/{camera_id}`** - Real-time video stream
- **`/ws/cameras`** - Multi-camera stream

## Usage

### Starting the Service
```bash
# Via service manager (recommended)
cd services
./start_services.sh --mode camera

# Direct startup
python3 camera/camera_service.py
```

### Jetson Device Setup
```bash
# On Jetson device
cd rover-2025/vision
./run_jetson.sh --device-id jetson-01
```

### Integration with UI
The React UI automatically connects to the camera service and displays video streams in the drive control interface.

## File Structure

```
services/camera/
├── camera_service.py         # Main FastAPI service
├── gstreamer_reader.py       # GStreamer video processing
├── aruco_detector.py         # ArUco marker detection
└── README.md                 # This documentation
```

## Dependencies

### System Dependencies
- **GStreamer** - Video processing framework
- **OpenCV** - Computer vision library
- **FastAPI** - Web framework
- **Uvicorn** - ASGI server

### Python Dependencies
- **opencv-contrib-python** - Computer vision
- **fastapi** - Web API framework
- **websockets** - Real-time communication
- **numpy** - Numerical processing

## Troubleshooting

### Common Issues
1. **No cameras detected** - Check Jetson server is running
2. **Video not streaming** - Verify network connectivity
3. **High latency** - Adjust buffer settings in config
4. **ArUco not detecting** - Check marker quality and lighting

### Debug Commands
```bash
# Check service status
curl http://localhost:8001/api/health

# List connected cameras
curl http://localhost:8001/api/cameras

# Check GStreamer pipeline
gst-launch-1.0 --gst-debug=3 [pipeline]
```

## Integration Notes

This camera service is designed to work seamlessly with:
- **Jetson devices** in the `vision/` folder
- **React UI** in the `robot-controller-ui/` folder
- **Service manager** in the parent `services/` folder

The service automatically adapts to available cameras and provides real-time video streaming to the teleop interface.

---

**Note**: This is the camera backend component of the teleop system. For Jetson/vision processing, see the `vision/` folder documentation.
