# GStreamer Integration for Multi-Camera System

This document explains the updated camera system that uses GStreamer for RTP/H.264 stream reception instead of manual UDP packet parsing.

## Overview

The system has been updated to:
1. **Receive RTP/H.264 streams** via UDP using GStreamer pipelines
2. **Decode frames** using OpenCV with GStreamer backend
3. **Process frames** with optional ArUco marker detection
4. **Stream to frontend** via WebSocket as JPEG/base64

## Architecture Changes

### Before (Old System)
```
Jetson → Raw H.264 UDP packets → Manual parsing → Frame reassembly → WebSocket
```

### After (New System)
```
Jetson → RTP/H.264 UDP → GStreamer pipeline → OpenCV frames → ArUco processing → WebSocket
```

## Key Components

### 1. GStreamerCameraReader (`gstreamer_reader.py`)
- Handles RTP/H.264 stream reception via GStreamer
- Provides decoded OpenCV frames
- Manages pipeline lifecycle and error recovery

**GStreamer Pipeline:**
```
udpsrc port=5004 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" ! 
rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink
```

### 2. Updated ArUco Detector (`aruco_detector.py`)
- Works directly with OpenCV frames (no H.264 decoding needed)
- Simplified API for frame processing
- Better performance and reliability

### 3. Updated Central Backend (`central_backend.py`)
- Uses GStreamer readers instead of UDP parsing
- Polling-based frame capture
- Dynamic camera management via REST API

## Usage

### Starting the Backend

```bash
# Start with default settings
python3 central_backend.py

# Start with custom settings
python3 central_backend.py --http-port 8001 --enable-aruco

# Disable ArUco detection
python3 central_backend.py --disable-aruco
```

### Default Camera Configuration

The backend automatically sets up these default cameras:
- `jetson-01-cam00` on port 5000 (Front Camera)
- `jetson-01-cam01` on port 5001 (Left Camera)  
- `jetson-01-cam02` on port 5002 (Right Camera)

### Adding/Removing Cameras via API

**Add a camera:**
```bash
curl -X POST http://localhost:8001/api/cameras/add \
  -H "Content-Type: application/json" \
  -d '{
    "camera_id": "new-camera",
    "port": 5003,
    "device_id": "jetson-02",
    "name": "Rear Camera"
  }'
```

**Remove a camera:**
```bash
curl -X DELETE http://localhost:8001/api/cameras/new-camera
```

**List cameras:**
```bash
curl http://localhost:8001/api/cameras
```

### Jetson/Pi Side Setup

Your Jetson should send RTP/H.264 streams using GStreamer:

```bash
# Example GStreamer command for Jetson
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=640,height=480,framerate=30/1 ! \
  videoconvert ! \
  x264enc tune=zerolatency bitrate=2000 speed-preset=superfast ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=BACKEND_IP port=5000
```

Replace `BACKEND_IP` with your backend server's IP address.

## Testing

### Run Integration Tests

```bash
# Run all tests
python3 test_integration.py

# Run specific tests
python3 test_integration.py --test aruco
python3 test_integration.py --test gstreamer --port 5004
python3 test_integration.py --test backend
python3 test_integration.py --test workflow
```

### Test GStreamer Reader Directly

```bash
# Test with actual RTP stream (if available)
python3 gstreamer_reader.py 5004 10

# Test ArUco detector with image
python3 aruco_detector.py path/to/image.jpg
```

## Configuration

### Camera Ports
Update the default camera ports in `central_backend.py`:

```python
default_cameras = [
    {"camera_id": "jetson-01-cam00", "port": 5000, "device_id": "jetson-01", "name": "Front Camera"},
    {"camera_id": "jetson-01-cam01", "port": 5001, "device_id": "jetson-01", "name": "Left Camera"},
    {"camera_id": "jetson-01-cam02", "port": 5002, "device_id": "jetson-01", "name": "Right Camera"},
]
```

### GStreamer Pipeline
Modify the pipeline in `gstreamer_reader.py` if needed:

```python
def _build_pipeline(self, port: int) -> str:
    pipeline = (
        f"udpsrc port={port} "
        f"caps=\"application/x-rtp,media=video,clock-rate=90000,"
        f"encoding-name=H264,payload=96\" ! "
        "rtph264depay ! "
        "h264parse ! "
        "avdec_h264 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=2"
    )
    return pipeline
```

## Troubleshooting

### Common Issues

1. **GStreamer pipeline fails to open**
   - Check if GStreamer is installed: `gst-launch-1.0 --version`
   - Verify the port is not in use: `netstat -an | grep :5004`
   - Check if RTP stream is being sent from Jetson

2. **No frames received**
   - Verify Jetson is sending to correct IP/port
   - Check network connectivity
   - Monitor with: `tcpdump -i any port 5004`

3. **ArUco detection not working**
   - Ensure markers are visible and well-lit
   - Check marker dictionary matches (DICT_4X4_100)
   - Verify camera focus and resolution

### Debug Commands

```bash
# Test GStreamer pipeline manually
gst-launch-1.0 -v udpsrc port=5004 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink sync=false

# Monitor network traffic
tcpdump -i any port 5004

# Check backend logs
python3 central_backend.py --http-port 8001 2>&1 | tee backend.log
```

## Performance Considerations

1. **Latency**: GStreamer pipeline adds ~50-100ms latency
2. **CPU Usage**: H.264 decoding is CPU-intensive
3. **Memory**: Each camera uses ~10-20MB RAM
4. **Network**: Each 640x480@30fps stream uses ~2Mbps

## Migration from Old System

### Changes Required

1. **Jetson/Pi side**: Switch from raw H.264 UDP to RTP/H.264
2. **Backend**: No manual changes needed (automatic)
3. **Frontend**: No changes needed (same WebSocket API)

### Backward Compatibility

The new system is **not backward compatible** with the old UDP packet format. All Jetson/Pi devices must be updated to send RTP streams.

## API Reference

### WebSocket API (unchanged)
- Connect: `ws://backend:8001/stream?camera_id=jetson-01-cam00`
- Message format: Same as before

### REST API (new endpoints)
- `GET /api/cameras` - List cameras
- `POST /api/cameras/add` - Add camera
- `DELETE /api/cameras/{camera_id}` - Remove camera
- `GET /api/health` - Health check

## Dependencies

Ensure these are installed:
- OpenCV with GStreamer support
- GStreamer 1.0 with plugins
- Python packages: `fastapi`, `uvicorn`, `websockets`, `numpy`

```bash
# Install GStreamer (Ubuntu/Debian)
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav

# Install Python packages
pip install fastapi uvicorn websockets numpy opencv-python
