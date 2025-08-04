// Multi-Camera System Configuration

// Central Backend Server Configuration
export const MULTI_CAMERA_BACKEND = {
  // Backend server host (where central_backend.py runs)
  HOST: "localhost", // Backend server IP (change to match your setup)
  
  // HTTP port for REST API and WebSocket connections
  HTTP_PORT: 8001,
  
  // UDP port for receiving frames from Jetson/Pi devices
  UDP_PORT: 9999,
  
  // WebSocket URL (constructed from above)
  get WEBSOCKET_URL() {
    return `ws://${this.HOST}:${this.HTTP_PORT}`;
  },
  
  // HTTP API base URL (constructed from above)
  get API_BASE_URL() {
    return `http://${this.HOST}:${this.HTTP_PORT}`;
  }
};

// Jetson/Pi Device Configuration
export const JETSON_DEVICES = {
  // Default backend host for Jetson/Pi devices to send frames to
  DEFAULT_BACKEND_HOST: "192.168.1.100", // Change to your backend server IP
  
  // Default backend UDP port for Jetson/Pi devices
  DEFAULT_BACKEND_PORT: 9999,
  
  // Default device configurations
  DEVICES: {
    JETSON_01: {
      DEVICE_ID: "jetson-01",
      DESCRIPTION: "Primary Jetson Device"
    },
    JETSON_02: {
      DEVICE_ID: "jetson-02", 
      DESCRIPTION: "Secondary Jetson Device"
    },
    RPI_01: {
      DEVICE_ID: "rpi-01",
      DESCRIPTION: "Raspberry Pi Device"
    }
  }
};

// Camera Stream Configuration
export const CAMERA_STREAM_CONFIG = {
  // Default JPEG quality (1-100)
  DEFAULT_JPEG_QUALITY: 80,
  
  // Default target FPS
  DEFAULT_FPS: 20,
  
  // WebSocket reconnection settings
  RECONNECT_DELAY: 2000, // milliseconds
  
  // Frame timeout (consider stream stalled after this time)
  FRAME_TIMEOUT: 5000, // milliseconds
  
  // Camera inactive timeout
  INACTIVE_TIMEOUT: 30.0, // seconds
  
  // Heartbeat timeout
  HEARTBEAT_TIMEOUT: 15.0, // seconds
  
  // Maximum number of cameras in multi-view grid
  MAX_GRID_CAMERAS: 4,
  
  // Frame buffer size per camera
  FRAME_BUFFER_SIZE: 10
};

// UI Configuration
export const CAMERA_UI_CONFIG = {
  // Auto-refresh interval for camera list
  CAMERA_LIST_REFRESH_INTERVAL: 10000, // milliseconds
  
  // Grid layout breakpoints
  GRID_LAYOUTS: {
    SINGLE: "grid-cols-1 grid-rows-1",
    DUAL: "grid-cols-2 grid-rows-1", 
    TRIPLE: "grid-cols-2 grid-rows-2",
    QUAD: "grid-cols-2 grid-rows-2"
  },
  
  // Default view mode
  DEFAULT_VIEW_MODE: "single" as "single" | "multi",
  
  // Show DPad for cameras with this name pattern
  DPAD_CAMERA_PATTERN: "USB 2.0 Camera"
};

// Legacy WebRTC Configuration (for backward compatibility)
export const LEGACY_WEBRTC_CONFIG = {
  // Base station IP for WebRTC signaling
  BASE_STATION_IP: "localhost",
  BASE_STATION_PORT: 8081,
  
  // Jetson IP for legacy camera discovery
  JETSON_IP: "192.168.1.69",
  JETSON_PORT: 8000,
  
  // WebRTC stream settings
  STALL_TIMEOUT: 3000, // milliseconds
  RESTART_DELAY: 500 // milliseconds
};

// Export combined configuration for easy access
export const CAMERA_CONFIG = {
  BACKEND: MULTI_CAMERA_BACKEND,
  DEVICES: JETSON_DEVICES,
  STREAM: CAMERA_STREAM_CONFIG,
  UI: CAMERA_UI_CONFIG,
  LEGACY: LEGACY_WEBRTC_CONFIG
};

// Helper functions
export const getCameraWebSocketUrl = (cameraId: string): string => {
  return `${MULTI_CAMERA_BACKEND.WEBSOCKET_URL}/stream?camera_id=${encodeURIComponent(cameraId)}`;
};

export const getCamerasApiUrl = (): string => {
  return `${MULTI_CAMERA_BACKEND.API_BASE_URL}/api/cameras`;
};

export const getHealthApiUrl = (): string => {
  return `${MULTI_CAMERA_BACKEND.API_BASE_URL}/api/health`;
};
