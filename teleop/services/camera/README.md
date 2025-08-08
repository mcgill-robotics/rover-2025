# Multi-Camera Streaming System

A full-stack low-latency video streaming system supporting multiple camera feeds from embedded devices (NVIDIA Jetson and Raspberry Pi). Uses UDP for frame transmission and WebSocket for frontend streaming.

## System Architecture

```
Jetson/Pi Devices    UDP Frames     Central Backend    WebSocket     React Frontend
┌─────────────────┐ ──────────────► ┌─────────────────┐ ────────────► ┌─────────────────┐
│ jetson_server.py│                 │central_backend  │               │MultiCameraView  │
│ - GStreamer cap │                 │ - Frame buffer  │               │ - Grid layout   │
│ - H.264 encoding│                 │ - ArUco detect  │               │ - Auto-reconnect│
│ - UDP streaming │                 │ - WebSocket API │               │ - Status display│
└─────────────────┘                 └─────────────────┘               └─────────────────┘
```

## Installation and Setup

### Step 1: Install Dependencies

On the central backend server:
```bash
cd teleop/services/camera
chmod +x setup.sh run_backend.sh run_jetson.sh
./setup.sh
```

On each Jetson/Pi device:
```bash
sudo apt update
sudo apt install python3-pip gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad v4l-utils
# For NVIDIA Jetson devices, also install:
sudo apt install gstreamer1.0-plugins-nvenc
```

### Step 2: Verify Camera Detection

On each Jetson/Pi device, check available cameras:
```bash
v4l2-ctl --list-devices
```

Expected output:
```
USB 2.0 Camera (usb-0000:01:00.0-1):
	/dev/video0
	/dev/video1

HD Pro Webcam C920 (usb-0000:01:00.0-2):
	/dev/video2
	/dev/video3
```

Test camera access:
```bash
v4l2-ctl --device=/dev/video0 --info
```

### Step 3: Configure Network Settings

Edit `config.py` to match your network (this is the single source of truth for all default values):
```python
BACKEND_CONFIG = {
    "HOST": "0.0.0.0",           # Backend listens on all interfaces
    "HTTP_PORT": 8001,           # WebSocket and API port
    "UDP_PORT": 9999,            # UDP frame reception port
    "INACTIVE_TIMEOUT": 30.0,    # Camera inactive timeout
    "HEARTBEAT_TIMEOUT": 15.0,   # Heartbeat timeout
}

JETSON_CONFIG = {
    "DEFAULT_BACKEND_HOST": "192.168.1.100",  # Update to backend IP
    "DEFAULT_BACKEND_PORT": 9999,             # Must match BACKEND_CONFIG["UDP_PORT"]
    "DEFAULT_JPEG_QUALITY": 80,               # JPEG compression quality
    "DEFAULT_FPS": 20,                        # Target frame rate
}
```

**Note**: The run scripts (`run_backend.sh` and `run_jetson.sh`) automatically load these default values from `config.py`. You can still override any value using command-line arguments.

Update frontend configuration in `teleop/robot-controller-ui/src/config/camera.ts`:
```typescript
export const CAMERA_CONFIG = {
  BACKEND: {
    BASE_URL: "http://192.168.1.100:8001",      // Update backend IP
    WEBSOCKET_URL: "ws://192.168.1.100:8001",   // Update backend IP
  },
};
```

## Running the System

### Step 4: Start Central Backend

On the backend server:
```bash
cd teleop/services/camera
./run_backend.sh
```

With custom options:
```bash
./run_backend.sh --http-port 8001 --udp-port 9999 --inactive-timeout 30
```

Verify backend is running:
```bash
curl http://localhost:8001/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "cameras_count": 0,
  "active_connections": 0,
  "timestamp": 1642678800.789
}
```

### Step 5: Start Camera Servers

On first Jetson/Pi device:
```bash
cd teleop/services/camera
./run_jetson.sh --device-id jetson-01
```

On second device:
```bash
./run_jetson.sh --device-id jetson-02
```

On Raspberry Pi:
```bash
./run_jetson.sh --device-id rpi-01
```

**Note**: The `--backend-host` is automatically loaded from `config.py` (192.168.1.100). Only specify it if you need to override the default:
```bash
./run_jetson.sh --backend-host 192.168.1.50 --device-id jetson-01
```

### Step 6: Verify Camera Registration

Check cameras are registered with backend:
```bash
curl http://192.168.1.100:8001/api/cameras
```

Expected response:
```json
{
  "cameras": [
    {
      "camera_id": "jetson-01-cam00",
      "device_id": "jetson-01",
      "name": "USB 2.0 Camera",
      "device_path": "/dev/video0",
      "is_active": true,
      "last_frame_time": 1642678800.123,
      "last_heartbeat_time": 1642678800.456
    }
  ]
}
```

### Step 7: Start Frontend

```bash
cd teleop/robot-controller-ui
npm install
npm run dev
```

Access the interface at `http://localhost:3000/drive`

## Command Line Options

### Backend Server Options

```bash
./run_backend.sh [OPTIONS]

--udp-port PORT           UDP port for frame reception (default: from config.py)
--http-port PORT          HTTP/WebSocket port (default: from config.py)
--inactive-timeout SEC    Camera inactive timeout (default: from config.py)
--heartbeat-timeout SEC   Heartbeat timeout (default: from config.py)
--enable-aruco            Enable ArUco marker detection (default)
--disable-aruco           Disable ArUco marker detection
```

### Jetson/Pi Server Options

```bash
./run_jetson.sh [OPTIONS]

--backend-host HOST       Backend server IP (default: from config.py)
--backend-port PORT       Backend UDP port (default: from config.py)
--device-id ID            Unique device identifier (default: jetson-01)
--jpeg-quality QUAL       JPEG quality 1-100 (default: from config.py)
--fps FPS                 Target frame rate (default: from config.py)
```

## Frontend Usage

### Single Camera Mode

1. Select camera from dropdown or use arrow buttons
2. Click Start button to begin streaming
3. Monitor FPS and connection status in top-right corner
4. Use arrow buttons to switch between cameras

### Multi-Camera Mode

1. Click "Multi View" button to switch modes
2. Select camera from dropdown menu
3. Click "Add Camera" to add to grid
4. Remove cameras using X button on each feed
5. Support for up to 4 simultaneous camera feeds

### Grid Layouts

- 1 camera: Full screen display
- 2 cameras: Side-by-side layout
- 3 cameras: One large (top) + two small (bottom)
- 4 cameras: 2x2 grid layout

## ArUco Marker Detection

The system includes built-in ArUco marker detection that automatically overlays detected markers on camera streams.

### Features

- **Real-time Detection**: ArUco markers are detected and highlighted in real-time on all camera feeds
- **Visual Overlay**: Detected markers are outlined with green borders and labeled with their IDs
- **Multiple Dictionaries**: Supports various ArUco dictionary types (4x4, 5x5, 6x6, 7x7)
- **Configurable**: Can be enabled/disabled via command line options

### Supported ArUco Dictionaries

- `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`
- `DICT_5X5_50`, `DICT_5X5_100`, `DICT_5X5_250`, `DICT_5X5_1000`
- `DICT_6X6_50`, `DICT_6X6_100`, `DICT_6X6_250`, `DICT_6X6_1000`
- `DICT_7X7_50`, `DICT_7X7_100`, `DICT_7X7_250`, `DICT_7X7_1000`

Default: `DICT_4X4_100`

### Backend Configuration

Enable ArUco detection (default):
```bash
./run_backend.sh --enable-aruco
```

Disable ArUco detection:
```bash
./run_backend.sh --disable-aruco
```

### Testing ArUco Detection

Generate test markers:
```bash
# Install OpenCV tools
pip3 install opencv-contrib-python

# Generate ArUco marker
python3 -c "
import cv2
import numpy as np

# Create ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Generate marker with ID 0
marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 200)

# Save marker
cv2.imwrite('aruco_marker_0.png', marker_img)
print('ArUco marker saved as aruco_marker_0.png')
"
```

Print the generated marker and hold it in front of a camera to test detection.

### WebSocket Frame Format with ArUco

When ArUco detection is enabled, WebSocket frame messages include an additional field:

```json
{
  "type": "frame",
  "camera_id": "jetson-01-cam00",
  "timestamp": 1642678800123,
  "received_time": 1642678800.456,
  "frame_data": "base64-encoded-jpeg-with-aruco-overlay",
  "aruco_detected": true
}
```

The `aruco_detected` field indicates whether any ArUco markers were found and processed in the frame.

## Debugging and Troubleshooting

### Camera Detection Issues

Problem: No cameras found
```bash
# Check camera devices exist
ls -la /dev/video*

# Verify v4l2-utils installed
sudo apt install v4l-utils

# Test camera access
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --all
```

Problem: Permission denied
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Logout and login again

# Or change permissions temporarily
sudo chmod 666 /dev/video*
```

### Network Connectivity Issues

Problem: Backend not reachable
```bash
# Test network connectivity
ping 192.168.1.100

# Check port accessibility
nc -zv 192.168.1.100 8001
nc -zuv 192.168.1.100 9999

# Check firewall
sudo ufw status
sudo ufw allow 8001
sudo ufw allow 9999/udp
```

Problem: UDP packets not received
```bash
# Monitor UDP traffic on backend
sudo tcpdump -i any port 9999

# Test UDP connectivity from Jetson
echo "test" | nc -u 192.168.1.100 9999
```

### Backend Server Issues

Problem: Backend crashes or fails to start
```bash
# Check Python dependencies
pip3 list | grep -E "(fastapi|uvicorn|opencv)"

# Run with debug logging
python3 central_backend.py --http-port 8001 --udp-port 9999

# Check port conflicts
sudo netstat -tulpn | grep -E "(8001|9999)"
```

Problem: WebSocket connection fails
```bash
# Test WebSocket manually
wscat -c ws://192.168.1.100:8001/stream?camera_id=jetson-01-cam00

# Check CORS settings in central_backend.py
# Verify allow_origins includes frontend domain
```

### Camera Server Issues

Problem: Camera capture fails
```bash
# Test camera directly with GStreamer
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=20/1 ! autovideosink

# Test GStreamer H.264 encoding
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=20/1 ! nvvidconv ! x264enc tune=zerolatency bitrate=512 ! h264parse ! fakesink

# Check GStreamer plugins
gst-inspect-1.0 v4l2src
gst-inspect-1.0 x264enc
```

Problem: High CPU usage
```bash
# Monitor system resources
htop

# Reduce frame rate and quality
./run_jetson.sh --fps 10 --jpeg-quality 60

# Check camera resolution
v4l2-ctl --device=/dev/video0 --get-fmt-video
```

### Frontend Issues

Problem: No cameras appear in dropdown
```bash
# Check API endpoint
curl http://192.168.1.100:8001/api/cameras

# Verify frontend config
grep -r "192.168.1.100" teleop/robot-controller-ui/src/config/
```

Problem: WebSocket connection fails
```bash
# Check browser console for errors
# Verify WebSocket URL in camera.ts config
# Test WebSocket connection manually
```

### Performance Issues

Problem: Low frame rate or high latency
```bash
# Monitor network bandwidth
iftop -i eth0

# Check system load
uptime
free -h

# Optimize settings
./run_jetson.sh --fps 15 --jpeg-quality 70
```

Problem: Memory usage grows over time
```bash
# Monitor memory usage
watch -n 1 'free -h && ps aux | grep -E "(central_backend|jetson_server)"'

# Check for memory leaks in logs
tail -f /var/log/syslog | grep -E "(camera|backend)"
```

## Log Analysis

### Backend Logs

```bash
# View backend logs
python3 central_backend.py 2>&1 | tee backend.log

# Common log messages
grep -E "(Camera|WebSocket|UDP)" backend.log
```

### Jetson Server Logs

```bash
# View camera server logs
python3 jetson_server.py 2>&1 | tee jetson.log

# Check camera discovery
grep "Found camera" jetson.log

# Check frame transmission
grep "Failed to send" jetson.log
```

## System Monitoring

### Health Check Script

Create `health_check.sh`:
```bash
#!/bin/bash
echo "=== Backend Health ==="
curl -s http://192.168.1.100:8001/api/health | jq .

echo "=== Active Cameras ==="
curl -s http://192.168.1.100:8001/api/cameras | jq '.cameras[] | {camera_id, is_active}'

echo "=== Network Connectivity ==="
nc -zv 192.168.1.100 8001 9999
```

### Performance Monitoring

```bash
# Monitor UDP packet rate
sudo tcpdump -i any port 9999 | pv -l > /dev/null

# Monitor WebSocket connections
ss -tuln | grep -E "(8001|9999)"

# Check system resources
iostat 1
vmstat 1
```

## Configuration Files

### Backend Configuration (`config.py`)

```python
BACKEND_CONFIG = {
    "HOST": "0.0.0.0",
    "HTTP_PORT": 8001,
    "UDP_PORT": 9999,
    "INACTIVE_TIMEOUT": 30.0,
    "HEARTBEAT_TIMEOUT": 15.0,
    "FRAME_BUFFER_SIZE": 10,
    "MAX_UDP_PACKET_SIZE": 65507,
}
```

### Frontend Configuration (`src/config/camera.ts`)

```typescript
export const CAMERA_CONFIG = {
  BACKEND: {
    BASE_URL: "http://192.168.1.100:8001",
    WEBSOCKET_URL: "ws://192.168.1.100:8001",
  },
  STREAM: {
    RECONNECT_DELAY: 2000,
    FRAME_TIMEOUT: 5000,
  },
};
```

## API Reference

### REST Endpoints

#### GET /api/cameras
Returns active camera list.

Response format:
```json
{
  "cameras": [
    {
      "camera_id": "jetson-01-cam00",
      "device_id": "jetson-01",
      "name": "USB 2.0 Camera",
      "device_path": "/dev/video0",
      "is_active": true,
      "last_frame_time": 1642678800.123,
      "last_heartbeat_time": 1642678800.456
    }
  ]
}
```

#### GET /api/health
System health status.

Response format:
```json
{
  "status": "healthy",
  "cameras_count": 3,
  "active_connections": 2,
  "timestamp": 1642678800.789
}
```

### WebSocket Protocol

#### Connection
```
ws://backend_ip:8001/stream?camera_id=jetson-01-cam00
```

#### Message Types

Frame message:
```json
{
  "type": "frame",
  "camera_id": "jetson-01-cam00",
  "timestamp": 1642678800123,
  "received_time": 1642678800.456,
  "frame_data": "base64-encoded-jpeg"
}
```

Status message:
```json
{
  "type": "status",
  "camera_id": "jetson-01-cam00",
  "message": "Connected"
}
```

Error message:
```json
{
  "type": "error",
  "message": "Camera jetson-01-cam00 not found"
}
```

## Security Considerations

Current limitations:
- No authentication required
- Unencrypted communication
- Open network access

Production recommendations:
- Implement JWT authentication
- Use HTTPS/WSS protocols
- Configure network firewalls
- Add rate limiting
- Enable access logging

## Development and Testing

### Manual Testing

Test camera discovery:
```bash
python3 -c "
from jetson_server import MultiCameraStreamer
streamer = MultiCameraStreamer('localhost', 9999, 'test-device')
cameras = streamer.discover_cameras()
for cam in cameras:
    print(f'{cam.camera_id}: {cam.name} ({cam.device_path})')
"
```

Test UDP transmission:
```bash
# Send test frame
echo "test frame data" | nc -u 192.168.1.100 9999

# Monitor backend reception
sudo tcpdump -i any -X port 9999
```

Test WebSocket connection:
```bash
# Install wscat if needed
npm install -g wscat

# Connect to stream
wscat -c "ws://192.168.1.100:8001/stream?camera_id=jetson-01-cam00"
```

### Code Modifications

To add new features:

1. Backend changes: Modify `central_backend.py`
2. Camera server changes: Modify `jetson_server.py`
3. Frontend changes: Update `MultiCameraView.tsx` and `useMultiCameraStream.ts`
4. Configuration: Update `config.py` and `camera.ts`

### Performance Tuning

Network optimization:
```bash
# Increase UDP buffer sizes
echo 'net.core.rmem_max = 134217728' >> /etc/sysctl.conf
echo 'net.core.rmem_default = 134217728' >> /etc/sysctl.conf
sysctl -p
```

Camera optimization:
```bash
# Set camera parameters
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=128
v4l2-ctl --device=/dev/video0 --set-ctrl=contrast=128
```

## Maintenance

### Regular Tasks

Daily:
- Check system logs for errors
- Monitor disk space usage
- Verify all cameras are active

Weekly:
- Restart services to clear memory
- Update system packages
- Check network performance

Monthly:
- Review and rotate log files
- Update camera firmware if available
- Performance benchmarking

### Backup and Recovery

Important files to backup:
- `config.py` - System configuration
- `teleop/robot-controller-ui/src/config/` - Frontend configuration
- Service scripts and documentation

Recovery procedure:
1. Restore configuration files
2. Reinstall dependencies using `setup.sh`
3. Restart all services
4. Verify camera registration and streaming
