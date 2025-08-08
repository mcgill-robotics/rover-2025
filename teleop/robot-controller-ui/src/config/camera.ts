// Multi-Camera System Configuration

// Central Backend Server Configuration
export const MULTI_CAMERA_BACKEND = {
  // Backend server host (where central_backend.py runs)
  HOST: "192.168.1.100", // Backend server IP (change to match your setup)
  
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

// Export combined configuration for easy access
export const CAMERA_CONFIG = {
  BACKEND: MULTI_CAMERA_BACKEND,
  STREAM: CAMERA_STREAM_CONFIG,
  UI: CAMERA_UI_CONFIG,
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
