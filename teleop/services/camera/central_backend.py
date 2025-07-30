#!/usr/bin/env python3
"""
Central backend server for multi-camera streaming system.
Receives UDP frames from Jetson/Pi devices and serves them via WebSocket to frontend.
"""

import asyncio
import json
import time
import threading
from collections import defaultdict, deque
from typing import Dict, Optional, Set
import logging
from dataclasses import dataclass
import base64
import cv2

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from config import get_backend_config
from aruco_detector import create_aruco_detector
from gstreamer_reader import GStreamerCameraReader, MultiGStreamerReader

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class CameraFrame:
    camera_id: str
    timestamp: int
    frame_data: bytes
    received_time: float

@dataclass
class CameraInfo:
    camera_id: str
    device_id: str
    name: str
    port: int
    is_active: bool
    last_frame_time: float
    resolution: tuple = (0, 0)
    fps: float = 0.0

class FrameBuffer:
    def __init__(self, max_size: int = 10):
        self.frames = deque(maxlen=max_size)
        self.lock = threading.Lock()

    def add_frame(self, frame: CameraFrame):
        with self.lock:
            self.frames.append(frame)

    def get_latest_frame(self) -> Optional[CameraFrame]:
        with self.lock:
            return self.frames[-1] if self.frames else None

    def clear(self):
        with self.lock:
            self.frames.clear()

class MultiCameraBackend:
    def __init__(self, http_port: int = 8001, enable_aruco: bool = True, loop: Optional[asyncio.AbstractEventLoop] = None):
        self.http_port = http_port
        self.enable_aruco = enable_aruco
        self.loop = loop or asyncio.get_event_loop()

        self.cameras: Dict[str, CameraInfo] = {}
        self.frame_buffers: Dict[str, FrameBuffer] = {}
        self.websocket_connections: Dict[str, Set[WebSocket]] = defaultdict(set)

        # GStreamer camera readers
        self.camera_readers = MultiGStreamerReader()
        self.running = False

        self.inactive_timeout = 30.0

        # Initialize ArUco detector
        self.aruco_detector = None
        if self.enable_aruco:
            try:
                config = get_backend_config()
                aruco_dict = config["ARUCO_CONFIG"]["DICTIONARY"]
                self.aruco_detector = create_aruco_detector(aruco_dict)
                logger.info(f"ArUco detector initialized with {aruco_dict}")
            except Exception as e:
                logger.error(f"Failed to initialize ArUco detector: {e}")
                self.enable_aruco = False

        self.app = FastAPI(title="Multi-Camera Backend", version="1.0.0")
        self.setup_routes()
        self.setup_cors()

    def setup_cors(self):
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    def setup_routes(self):
        @self.app.get("/api/cameras")
        async def get_cameras():
            current_time = time.time()
            active_cameras = []
            for camera_id, info in self.cameras.items():
                if current_time - info.last_frame_time < self.inactive_timeout:
                    active_cameras.append({**info.__dict__})
            return {"cameras": active_cameras}

        @self.app.post("/api/cameras/add")
        async def add_camera(camera_data: dict):
            """Add a new camera with specified port."""
            try:
                camera_id = camera_data.get("camera_id")
                port = camera_data.get("port")
                device_id = camera_data.get("device_id", "unknown")
                name = camera_data.get("name", camera_id)
                
                if not camera_id or not port:
                    return {"error": "camera_id and port are required"}
                
                # Add camera reader
                reader = self.camera_readers.add_camera(camera_id, port)
                
                # Create camera info
                self.cameras[camera_id] = CameraInfo(
                    camera_id=camera_id,
                    device_id=device_id,
                    name=name,
                    port=port,
                    is_active=reader.is_active(),
                    last_frame_time=time.time(),
                    resolution=reader.get_resolution(),
                    fps=reader.get_frame_rate()
                )
                
                # Create frame buffer
                self.frame_buffers[camera_id] = FrameBuffer()
                
                logger.info(f"Added camera {camera_id} on port {port}")
                return {"success": True, "camera_id": camera_id, "port": port}
                
            except Exception as e:
                logger.error(f"Failed to add camera: {e}")
                return {"error": str(e)}

        @self.app.delete("/api/cameras/{camera_id}")
        async def remove_camera(camera_id: str):
            """Remove a camera."""
            try:
                if camera_id in self.cameras:
                    # Remove camera reader
                    self.camera_readers.remove_camera(camera_id)
                    
                    # Remove camera info and buffer
                    del self.cameras[camera_id]
                    self.frame_buffers.pop(camera_id, None)
                    
                    # Close WebSocket connections
                    connections = self.websocket_connections.pop(camera_id, set())
                    for ws in connections:
                        await ws.close()
                    
                    logger.info(f"Removed camera {camera_id}")
                    return {"success": True, "camera_id": camera_id}
                else:
                    return {"error": f"Camera {camera_id} not found"}
                    
            except Exception as e:
                logger.error(f"Failed to remove camera: {e}")
                return {"error": str(e)}

        @self.app.websocket("/stream")
        async def websocket_stream(websocket: WebSocket, camera_id: str):
            await websocket.accept()
            if camera_id not in self.cameras:
                await websocket.send_json({"type": "error", "message": f"Camera {camera_id} not found"})
                await websocket.close()
                return
            self.websocket_connections[camera_id].add(websocket)
            logger.info(f"WebSocket connected for camera {camera_id}")
            try:
                await websocket.send_json({"type": "status", "camera_id": camera_id, "message": "Connected"})
                while True:
                    try:
                        message = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                        data = json.loads(message)
                        if data.get("type") == "ping":
                            await websocket.send_json({"type": "pong"})
                    except asyncio.TimeoutError:
                        pass
                    except WebSocketDisconnect:
                        break
            finally:
                self.websocket_connections[camera_id].discard(websocket)
                logger.info(f"WebSocket disconnected for camera {camera_id}")

        @self.app.get("/api/health")
        async def health_check():
            return {
                "status": "healthy",
                "cameras_count": len(self.cameras),
                "active_connections": sum(len(v) for v in self.websocket_connections.values()),
                "timestamp": time.time()
            }

    def add_default_cameras(self):
        """Add default cameras based on configuration."""
        try:
            # Get default camera configuration from config
            config = get_backend_config()
            default_cameras = config["DEFAULT_CAMERAS"]
            
            for cam_config in default_cameras:
                try:
                    camera_id = cam_config["camera_id"]
                    port = cam_config["port"]
                    
                    # Add camera reader
                    reader = self.camera_readers.add_camera(camera_id, port)
                    
                    # Create camera info
                    self.cameras[camera_id] = CameraInfo(
                        camera_id=camera_id,
                        device_id=cam_config["device_id"],
                        name=cam_config["name"],
                        port=port,
                        is_active=reader.is_active(),
                        last_frame_time=time.time(),
                        resolution=reader.get_resolution(),
                        fps=reader.get_frame_rate()
                    )
                    
                    # Create frame buffer
                    self.frame_buffers[camera_id] = FrameBuffer()
                    
                    logger.info(f"Added default camera {camera_id} on port {port}")
                    
                except Exception as e:
                    logger.warning(f"Failed to add default camera {cam_config['camera_id']}: {e}")
                    
        except Exception as e:
            logger.error(f"Failed to add default cameras: {e}")

    async def broadcast_frame_data(self, camera_id: str, message: dict):
        """Broadcast frame data to connected WebSocket clients."""
        connections = self.websocket_connections.get(camera_id, set()).copy()
        if not connections:
            return
            
        for ws in connections:
            try:
                await ws.send_json(message)
            except Exception as e:
                logger.warning(f"WebSocket send failed: {e}")
                self.websocket_connections[camera_id].discard(ws)

    def camera_polling_loop(self, camera_id: str, reader: GStreamerCameraReader):
        """Polling loop for a single camera."""
        logger.info(f"Starting polling loop for camera {camera_id}")
        
        while self.running:
            try:
                frame = reader.read_frame()
                if frame is not None:
                    # Update camera info
                    if camera_id in self.cameras:
                        self.cameras[camera_id].last_frame_time = time.time()
                        self.cameras[camera_id].is_active = True
                    
                    # Process frame with ArUco detection if enabled
                    processed_frame = frame
                    aruco_detected = False
                    
                    if self.enable_aruco and self.aruco_detector:
                        try:
                            processed_frame, aruco_detected = self.aruco_detector.process_frame(frame)
                        except Exception as e:
                            logger.error(f"ArUco detection failed for {camera_id}: {e}")
                    
                    # Encode frame to JPEG
                    try:
                        config = get_backend_config()
                        jpeg_quality = config["JPEG_CONFIG"]["QUALITY"]
                        _, encoded = cv2.imencode(".jpg", processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
                        b64 = base64.b64encode(encoded).decode("utf-8")
                        
                        # Create message
                        message = {
                            "type": "frame",
                            "camera_id": camera_id,
                            "timestamp": int(time.time() * 1000),
                            "received_time": time.time(),
                            "frame_data": b64,
                            "aruco_detected": aruco_detected
                        }
                        
                        # Store in frame buffer
                        if camera_id in self.frame_buffers:
                            frame_obj = CameraFrame(camera_id, message["timestamp"], encoded.tobytes(), message["received_time"])
                            self.frame_buffers[camera_id].add_frame(frame_obj)
                        
                        # Broadcast to WebSocket clients
                        asyncio.run_coroutine_threadsafe(
                            self.broadcast_frame_data(camera_id, message), self.loop
                        )
                        
                    except Exception as e:
                        logger.error(f"Failed to encode frame for {camera_id}: {e}")
                        
                else:
                    # No frame received, brief sleep
                    time.sleep(0.033)  # ~30 FPS
                    
            except Exception as e:
                logger.error(f"Error in camera polling loop for {camera_id}: {e}")
                time.sleep(1)  # Longer sleep on error
                
        logger.info(f"Camera polling loop ended for {camera_id}")

    def start_camera_polling_threads(self):
        """Start polling threads for all cameras."""
        for camera_id, reader in self.camera_readers.get_all_readers().items():
            thread = threading.Thread(
                target=self.camera_polling_loop, 
                args=(camera_id, reader), 
                daemon=True
            )
            thread.start()
            logger.info(f"Started polling thread for camera {camera_id}")

    def cleanup_thread(self):
        """Cleanup thread to remove inactive cameras."""
        logger.info("Starting cleanup thread")
        while self.running:
            try:
                now = time.time()
                for cam_id in list(self.cameras):
                    info = self.cameras[cam_id]
                    if now - info.last_frame_time > self.inactive_timeout:
                        logger.info(f"Removing inactive camera: {cam_id}")
                        
                        # Remove camera reader
                        self.camera_readers.remove_camera(cam_id)
                        
                        # Remove camera info and buffer
                        del self.cameras[cam_id]
                        self.frame_buffers.pop(cam_id, None)
                        
                        # Close WebSocket connections
                        conns = self.websocket_connections.pop(cam_id, set())
                        for ws in conns:
                            asyncio.run_coroutine_threadsafe(ws.close(), self.loop)
                            
                time.sleep(10)
            except Exception as e:
                logger.error(f"Cleanup error: {e}")
                time.sleep(10)

    def run(self):
        """Start the multi-camera backend server."""
        logger.info("Starting multi-camera backend server")
        self.running = True
        
        # Add default cameras
        self.add_default_cameras()
        
        # Start camera polling threads
        self.start_camera_polling_threads()
        
        # Start cleanup thread
        threading.Thread(target=self.cleanup_thread, daemon=True).start()
        
        # Start the web server
        try:
            uvicorn.run(self.app, host="0.0.0.0", port=self.http_port, log_level="info")
        finally:
            self.running = False
            self.camera_readers.release_all()


def main():
    import argparse
    config = get_backend_config()
    parser = argparse.ArgumentParser()
    parser.add_argument('--http-port', type=int, default=config["HTTP_PORT"])
    parser.add_argument('--inactive-timeout', type=float, default=config["INACTIVE_TIMEOUT"])
    parser.add_argument('--enable-aruco', action='store_true', default=True)
    parser.add_argument('--disable-aruco', action='store_true', default=False)
    args = parser.parse_args()

    enable_aruco = args.enable_aruco and not args.disable_aruco

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    backend = MultiCameraBackend(
        http_port=args.http_port,
        enable_aruco=enable_aruco,
        loop=loop
    )
    backend.inactive_timeout = args.inactive_timeout
    backend.run()


if __name__ == "__main__":
    main()
