# Vision Server

This directory contains the lightweight vision server for Jetson/Pi devices in the Rover 2025 multi-camera system.

## Overview

The vision server consists of:
- **Vision Server**: Multi-camera capture and RTP streaming
- **Device Management**: Camera discovery and configuration
- **GStreamer Integration**: H.264 encoding and RTP streaming

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Jetson/Pi     │    │   Network       │    │   Teleop        │
│   Cameras       │───►│   (RTP/H.264)   │───►│   Backend       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Camera Input   │    │  RTP Streams    │    │  Camera Service │
│  (v4l2)        │    │  (Ports 5000-2) │    │  (Web API)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Features

- ✅ **Multi-Camera Support** - Up to 3 cameras (front, left, right)
- ✅ **GStreamer Integration** - H.264 encoding with low latency
- ✅ **RTP Streaming** - Real-time video streams to backend
- ✅ **Camera Discovery** - Automatic v4l2 camera detection
- ✅ **Configurable Settings** - FPS, resolution, bitrate
- ✅ **Device Management** - Unique device identification

## Setup Instructions

### 1. Install Dependencies

```bash
# Navigate to vision directory
cd rover-2025/vision

# Install dependencies
./setup_env.sh

# Install system dependencies (Jetson)
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
sudo apt-get install gstreamer1.0-plugins-nvvideo4linux2
sudo apt-get install v4l-utils
```

### 2. Configure Camera Settings

Edit `config.yml` to customize:
- Backend host and port
- Camera resolution and FPS
- H.264 encoding settings
- Network timeouts

### 3. Run Vision Server

```bash
# Basic usage
./run_vision_server.sh --device-id device-01

# With custom settings
./run_vision_server.sh \
    --backend-host 192.168.1.100 \
    --device-id device-01 \
    --fps 30 \
    --width 1280 \
    --height 720
```

## Configuration

### Vision Server Configuration

```yaml
jetson_config:
  default_backend_host: "192.168.1.100"
  default_fps: 20
  capture_width: 640
  capture_height: 480
  gstreamer_config:
    h264_bitrate: 512  # kbps
    h264_tune: "zerolatency"
```

### Dynamic Port Assignment

The vision server automatically chooses any available port for each camera:
- **No Fixed Ports**: Each camera gets assigned to any available port
- **Automatic Discovery**: Backend automatically discovers and connects to cameras
- **Scalable**: Can handle any number of cameras without configuration

## Usage

### Command Line Options

```bash
./run_jetson.sh [OPTIONS]

Options:
  --backend-host HOST    Backend server IP address
  --backend-port PORT    Backend server UDP port
  --device-id ID         Unique device identifier
  --width WIDTH          Video capture width
  --height HEIGHT        Video capture height
  --fps FPS              Target frames per second
  --bitrate KBPS         H.264 bitrate in kbps
  --tune PRESET          H.264 encoder tune preset
```

### Example Usage

```bash
# Start with default settings
./run_jetson.sh --device-id jetson-01

# High-quality streaming
./run_jetson.sh \
    --device-id jetson-01 \
    --fps 30 \
    --width 1280 \
    --height 720 \
    --bitrate 1024

# Multiple devices
./run_jetson.sh --device-id jetson-02 --backend-host 192.168.1.101
```

## Troubleshooting

### Common Issues

1. **GStreamer not found**:
   ```bash
   sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
   ```

2. **No cameras detected**:
   ```bash
   v4l2-ctl --list-devices
   ls /dev/video*
   ```

3. **Network connectivity**:
   ```bash
   ping 192.168.1.100
   netstat -an | grep 5000
   ```

### Performance Tuning

- **Lower latency**: Use `--tune zerolatency`
- **Higher quality**: Increase bitrate with `--bitrate 1024`
- **Better performance**: Use hardware encoding on Jetson

## Development

### Adding New Features

1. **New camera types**: Modify `jetson_server.py`
2. **Different encoders**: Update GStreamer pipeline
3. **Additional protocols**: Add new streaming methods

### Testing

```bash
# Run tests
pytest tests/

# Test camera detection
python3 -c "from jetson_server import discover_cameras; print(discover_cameras())"
```

## Integration

This vision system integrates with:
- **Teleop Backend**: Receives RTP streams
- **ROS System**: Can publish camera topics
- **Web Interface**: Displays camera feeds

---

**Note**: This is the Jetson/vision side of the camera system. The backend service is managed by the teleop system. 