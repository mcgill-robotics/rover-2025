import { useCallback, useEffect, useRef, useState } from "react";
import { CAMERA_CONFIG } from "@/config/camera";

export interface CameraInfo {
  camera_id: string;
  device_id: string;
  name: string;
  device_path: string;
  is_active: boolean;
  last_frame_time: number;
  last_heartbeat_time: number;
}

export interface CameraStreamState {
  isConnected: boolean;
  isReceivingFrames: boolean;
  lastFrameTime: number;
  fps: number;
  error: string | null;
}

export interface MultiCameraStreamOptions {
  backendUrl?: string;
  reconnectDelay?: number;
  frameTimeout?: number;
}

export function useMultiCameraStream({
  backendUrl = CAMERA_CONFIG.BACKEND.WEBSOCKET_URL,
  reconnectDelay = CAMERA_CONFIG.STREAM.RECONNECT_DELAY,
  frameTimeout = CAMERA_CONFIG.STREAM.FRAME_TIMEOUT,
}: MultiCameraStreamOptions = {}) {
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [streamStates, setStreamStates] = useState<Record<string, CameraStreamState>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // WebSocket connections for each camera
  const websockets = useRef<Record<string, WebSocket>>({});
  const canvasRefs = useRef<Record<string, HTMLCanvasElement>>({});
  const frameTimestamps = useRef<Record<string, number[]>>({});
  const reconnectTimeouts = useRef<Record<string, NodeJS.Timeout>>({});

  // Fetch available cameras
  const fetchCameras = useCallback(async () => {
    setIsLoading(true);
    setError(null);
    
    try {
      const response = await fetch(`${backendUrl.replace('ws://', 'http://')}/api/cameras`);
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }
      
      const data = await response.json();
      setCameras(data.cameras || []);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch cameras';
      setError(errorMessage);
      console.error('Failed to fetch cameras:', err);
    } finally {
      setIsLoading(false);
    }
  }, [backendUrl]);

  // Initialize stream state for a camera
  const initializeStreamState = useCallback((cameraId: string) => {
    setStreamStates(prev => ({
      ...prev,
      [cameraId]: {
        isConnected: false,
        isReceivingFrames: false,
        lastFrameTime: 0,
        fps: 0,
        error: null,
      }
    }));
    frameTimestamps.current[cameraId] = [];
  }, []);

  // Update FPS calculation for a camera
  const updateFPS = useCallback((cameraId: string) => {
    const timestamps = frameTimestamps.current[cameraId] || [];
    if (timestamps.length < 2) {
      setStreamStates(prev => ({
        ...prev,
        [cameraId]: { ...prev[cameraId], fps: 0 }
      }));
      return;
    }

    const now = Date.now();
    const recentTimestamps = timestamps.filter(t => now - t <= 2000);
    frameTimestamps.current[cameraId] = recentTimestamps;

    if (recentTimestamps.length > 1) {
      const timeSpan = (recentTimestamps[recentTimestamps.length - 1] - recentTimestamps[0]) / 1000;
      const fps = Math.round((recentTimestamps.length - 1) / timeSpan * 10) / 10;
      
      setStreamStates(prev => ({
        ...prev,
        [cameraId]: { ...prev[cameraId], fps }
      }));
    }
  }, []);

  // Connect to a camera stream
  const connectCamera = useCallback((cameraId: string) => {
    if (websockets.current[cameraId]) {
      return; // Already connected
    }

    initializeStreamState(cameraId);

    const wsUrl = `${backendUrl}/stream?camera_id=${encodeURIComponent(cameraId)}`;
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      console.log(`WebSocket connected for camera ${cameraId}`);
      setStreamStates(prev => ({
        ...prev,
        [cameraId]: { ...prev[cameraId], isConnected: true, error: null }
      }));
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        if (data.type === 'frame') {
          const now = Date.now();
          
          // Update frame timestamps
          if (!frameTimestamps.current[cameraId]) {
            frameTimestamps.current[cameraId] = [];
          }
          frameTimestamps.current[cameraId].push(now);
          
          // Update stream state
          setStreamStates(prev => ({
            ...prev,
            [cameraId]: {
              ...prev[cameraId],
              isReceivingFrames: true,
              lastFrameTime: now,
              error: null
            }
          }));

          // Decode and display frame
          const canvas = canvasRefs.current[cameraId];
          if (canvas) {
            const ctx = canvas.getContext('2d');
            if (ctx) {
              const img = new Image();
              img.onload = () => {
                canvas.width = img.width;
                canvas.height = img.height;
                ctx.drawImage(img, 0, 0);
              };
              img.src = `data:image/jpeg;base64,${data.frame_data}`;
            }
          }

          // Update FPS
          updateFPS(cameraId);
        } else if (data.type === 'error') {
          setStreamStates(prev => ({
            ...prev,
            [cameraId]: { ...prev[cameraId], error: data.message }
          }));
        }
      } catch (err) {
        console.error(`Error processing message for camera ${cameraId}:`, err);
      }
    };

    ws.onerror = (error) => {
      console.error(`WebSocket error for camera ${cameraId}:`, error);
      setStreamStates(prev => ({
        ...prev,
        [cameraId]: { ...prev[cameraId], error: 'Connection error' }
      }));
    };

    ws.onclose = (event) => {
      console.log(`WebSocket closed for camera ${cameraId}:`, event.code, event.reason);
      
      setStreamStates(prev => ({
        ...prev,
        [cameraId]: {
          ...prev[cameraId],
          isConnected: false,
          isReceivingFrames: false,
          error: event.code === 1000 ? null : 'Connection lost'
        }
      }));

      // Clean up
      delete websockets.current[cameraId];
      
      // Auto-reconnect if not a normal closure
      if (event.code !== 1000 && !reconnectTimeouts.current[cameraId]) {
        reconnectTimeouts.current[cameraId] = setTimeout(() => {
          delete reconnectTimeouts.current[cameraId];
          connectCamera(cameraId);
        }, reconnectDelay);
      }
    };

    websockets.current[cameraId] = ws;
  }, [backendUrl, initializeStreamState, updateFPS, reconnectDelay]);

  // Disconnect from a camera stream
  const disconnectCamera = useCallback((cameraId: string) => {
    const ws = websockets.current[cameraId];
    if (ws) {
      ws.close(1000, 'User disconnected');
      delete websockets.current[cameraId];
    }

    // Clear reconnect timeout
    if (reconnectTimeouts.current[cameraId]) {
      clearTimeout(reconnectTimeouts.current[cameraId]);
      delete reconnectTimeouts.current[cameraId];
    }

    // Clear frame timestamps
    delete frameTimestamps.current[cameraId];

    // Update state
    setStreamStates(prev => {
      const newState = { ...prev };
      delete newState[cameraId];
      return newState;
    });
  }, []);

  // Register a canvas for a camera
  const registerCanvas = useCallback((cameraId: string, canvas: HTMLCanvasElement | null) => {
    if (canvas) {
      canvasRefs.current[cameraId] = canvas;
    } else {
      delete canvasRefs.current[cameraId];
    }
  }, []);

  // Check for stalled streams
  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();
      
      setStreamStates(prev => {
        const newState = { ...prev };
        
        Object.keys(newState).forEach(cameraId => {
          const state = newState[cameraId];
          if (state.isConnected && state.lastFrameTime > 0) {
            const timeSinceLastFrame = now - state.lastFrameTime;
            if (timeSinceLastFrame > frameTimeout) {
              newState[cameraId] = {
                ...state,
                isReceivingFrames: false,
                error: 'Stream timeout'
              };
            }
          }
        });
        
        return newState;
      });
    }, 1000);

    return () => clearInterval(interval);
  }, [frameTimeout]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      Object.values(websockets.current).forEach(ws => ws.close());
      Object.values(reconnectTimeouts.current).forEach(timeout => clearTimeout(timeout));
    };
  }, []);

  // Auto-fetch cameras on mount
  useEffect(() => {
    fetchCameras();
  }, [fetchCameras]);

  return {
    cameras,
    streamStates,
    isLoading,
    error,
    fetchCameras,
    connectCamera,
    disconnectCamera,
    registerCanvas,
  };
}
