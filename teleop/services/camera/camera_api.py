#!/usr/bin/env python3
"""
FastAPI endpoints for the multi-camera streaming system.
Handles camera management, streaming, and settings updates.
"""

import asyncio
import base64
import json
import logging
import time
from typing import Dict, Optional, Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CameraAPI:
    def __init__(self, backend):
        """Initialize the camera API with a reference to the backend."""
        self.backend = backend
        self.app = FastAPI(title="Multi-Camera API", version="1.0.0")
        self.setup_cors()
        self.setup_routes()

    def setup_cors(self):
        """Configure CORS middleware."""
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    def setup_routes(self):
        """Setup all API routes."""
        @self.app.get("/api/cameras")
        async def get_cameras():
            """Get currently active cameras."""
            current_time = time.time()
            active_cameras = []
            for camera_id, info in self.backend.cameras.items():
                if current_time - info.last_frame_time < self.backend.inactive_timeout:
                    active_cameras.append({**info.__dict__})
            return {"cameras": active_cameras}

        @self.app.get("/api/cameras/available")
        async def get_available_cameras():
            """Get all available cameras from connected devices."""
            available_cameras = []
            current_time = time.time()
            
            for device_id, device_info in self.backend.jetson_devices.items():
                # Only include devices that have sent recent heartbeats
                if current_time - device_info['last_heartbeat'] < 30:
                    cameras_data = device_info.get('cameras', {})
                    
                    for camera_id, camera_info in cameras_data.items():
                        available_cameras.append({
                            "camera_id": camera_id,
                            "device_id": device_id,
                            "name": camera_info.get('name', camera_id),
                            "device_path": camera_info.get('device_path', ''),
                            "is_active": camera_info.get('is_active', False),
                            "rtp_port": camera_info.get('rtp_port'),
                            "host": device_info['host'],
                            "last_heartbeat": device_info['last_heartbeat'],
                            "status": "connected"
                        })
            
            return {
                "available_cameras": available_cameras, 
                "total_devices": len(self.backend.jetson_devices),
                "total_cameras": len(available_cameras)
            }

        @self.app.post("/api/cameras/{camera_id}/start")
        async def start_camera(camera_id: str):
            """Request to start a specific camera."""
            try:
                # Find which device this camera belongs to
                device_id = None
                for cam_id, cam_info in self.backend.cameras.items():
                    if cam_id == camera_id:
                        device_id = cam_info.device_id
                        break
                
                # If camera not found in active cameras, try to find it from device info
                if not device_id:
                    parts = camera_id.split('-')
                    if len(parts) >= 2:
                        potential_device_id = '-'.join(parts[:-1])
                        if potential_device_id in self.backend.jetson_devices:
                            device_id = potential_device_id
                
                if not device_id:
                    return {"error": f"Cannot determine device for camera {camera_id}"}
                
                command = {
                    "type": "start_camera",
                    "camera_id": camera_id
                }
                
                success = self.backend.send_command_to_jetson(device_id, command)
                
                if success:
                    return {"success": True, "message": f"Start command sent for camera {camera_id}"}
                else:
                    return {"error": f"Failed to send start command for camera {camera_id}"}
                    
            except Exception as e:
                logger.error(f"Failed to start camera {camera_id}: {e}")
                return {"error": str(e)}

        @self.app.post("/api/cameras/{camera_id}/stop")
        async def stop_camera(camera_id: str):
            """Request to stop a specific camera."""
            try:
                device_id = None
                if camera_id in self.backend.cameras:
                    device_id = self.backend.cameras[camera_id].device_id
                else:
                    parts = camera_id.split('-')
                    if len(parts) >= 2:
                        potential_device_id = '-'.join(parts[:-1])
                        if potential_device_id in self.backend.jetson_devices:
                            device_id = potential_device_id
                
                if not device_id:
                    return {"error": f"Cannot determine device for camera {camera_id}"}
                
                command = {
                    "type": "stop_camera",
                    "camera_id": camera_id
                }
                
                success = self.backend.send_command_to_jetson(device_id, command)
                
                if success:
                    return {"success": True, "message": f"Stop command sent for camera {camera_id}"}
                else:
                    return {"error": f"Failed to send stop command for camera {camera_id}"}
                    
            except Exception as e:
                logger.error(f"Failed to stop camera {camera_id}: {e}")
                return {"error": str(e)}

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
                reader = self.backend.camera_readers.add_camera(camera_id, port)
                
                # Create camera info
                self.backend.cameras[camera_id] = self.backend.CameraInfo(
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
                self.backend.frame_buffers[camera_id] = self.backend.FrameBuffer()
                
                logger.info(f"Added camera {camera_id} on port {port}")
                return {"success": True, "camera_id": camera_id, "port": port}
                
            except Exception as e:
                logger.error(f"Failed to add camera: {e}")
                return {"error": str(e)}

        @self.app.delete("/api/cameras/{camera_id}")
        async def remove_camera(camera_id: str):
            """Remove a camera."""
            try:
                if camera_id in self.backend.cameras:
                    # Remove camera reader
                    self.backend.camera_readers.remove_camera(camera_id)
                    
                    # Remove camera info and buffer
                    del self.backend.cameras[camera_id]
                    self.backend.frame_buffers.pop(camera_id, None)
                    
                    # Close WebSocket connections
                    connections = self.backend.websocket_connections.pop(camera_id, set())
                    for ws in connections:
                        await ws.close()
                    
                    logger.info(f"Removed camera {camera_id}")
                    return {"success": True, "camera_id": camera_id}
                else:
                    return {"error": f"Camera {camera_id} not found"}
                    
            except Exception as e:
                logger.error(f"Failed to remove camera: {e}")
                return {"error": str(e)}

        @self.app.post("/api/cameras/{camera_id}/settings")
        async def update_camera_settings(camera_id: str, settings: dict):
            """Update camera settings (bitrate, fps, etc.)."""
            try:
                if camera_id not in self.backend.cameras:
                    return {"success": False, "message": f"Camera {camera_id} not found"}
                
                camera = self.backend.cameras[camera_id]
                if not camera.is_active:
                    return {"success": False, "message": f"Camera {camera_id} is not active"}
                
                # Extract settings
                bitrate = settings.get('bitrate')
                fps = settings.get('fps')
                
                # Build command to send to device
                command = {
                    "type": "update_settings",
                    "camera_id": camera_id,
                    "settings": {}
                }
                
                if bitrate is not None:
                    command["settings"]["bitrate"] = bitrate
                    logger.info(f"Updating bitrate for camera {camera_id} to {bitrate} kbps")
                
                if fps is not None:
                    command["settings"]["fps"] = fps
                    logger.info(f"Updating FPS for camera {camera_id} to {fps}")
                
                # Find the device that owns this camera
                device_id = None
                for cam in self.backend.available_cameras:
                    if cam.camera_id == camera_id:
                        device_id = cam.device_id
                        break
                
                if device_id:
                    # Send command to the device
                    self.backend.send_command_to_jetson(device_id, command)
                    
                    # Update local camera info
                    if bitrate is not None:
                        camera.bitrate = bitrate
                    if fps is not None:
                        camera.fps = fps
                    
                    return {
                        "success": True, 
                        "message": f"Settings updated for camera {camera_id}",
                        "settings": command["settings"]
                    }
                else:
                    return {"success": False, "message": f"Device not found for camera {camera_id}"}
                    
            except Exception as e:
                logger.error(f"Error updating camera settings for {camera_id}: {e}")
                return {"success": False, "message": f"Error updating settings: {str(e)}"}

        @self.app.patch("/api/cameras/{camera_id}/bitrate")
        async def update_bitrate_dynamic(camera_id: str, data: dict):
            """Dynamically change bitrate without restarting pipeline."""
            try:
                if camera_id not in self.backend.cameras:
                    return {"success": False, "message": f"Camera {camera_id} not found"}
                
                camera = self.backend.cameras[camera_id]
                if not camera.is_active:
                    return {"success": False, "message": f"Camera {camera_id} is not active"}
                
                bitrate = data.get('bitrate')
                if bitrate is None:
                    return {"success": False, "message": "Bitrate value is required"}
                
                if not (128 <= bitrate <= 2048):
                    return {"success": False, "message": "Bitrate must be between 128 and 2048 kbps"}
                
                # Build dynamic update command
                command = {
                    "type": "dynamic_update",
                    "camera_id": camera_id,
                    "property": "bitrate",
                    "value": bitrate
                }
                
                # Find the device that owns this camera
                device_id = None
                for cam in self.backend.available_cameras:
                    if cam.camera_id == camera_id:
                        device_id = cam.device_id
                        break
                
                if device_id:
                    # Send dynamic update command
                    self.backend.send_command_to_jetson(device_id, command)
                    
                    # Update local camera info immediately
                    camera.bitrate = bitrate
                    
                    logger.info(f"Dynamically updated bitrate for camera {camera_id} to {bitrate} kbps")
                    
                    return {
                        "success": True, 
                        "message": f"Bitrate dynamically updated to {bitrate} kbps",
                        "bitrate": bitrate
                    }
                else:
                    return {"success": False, "message": f"Device not found for camera {camera_id}"}
                    
            except Exception as e:
                logger.error(f"Error updating bitrate dynamically for {camera_id}: {e}")
                return {"success": False, "message": f"Error updating bitrate: {str(e)}"}

        @self.app.websocket("/stream")
        async def websocket_stream(websocket: WebSocket, camera_id: str = None):
            await websocket.accept()
            
            # Get camera_id from query parameters if not provided as path parameter
            if not camera_id:
                query_params = dict(websocket.query_params)
                camera_id = query_params.get('camera_id')
            
            if not camera_id:
                await websocket.send_json({
                    "type": "error",
                    "message": "camera_id parameter is required"
                })
                await websocket.close()
                return

            if camera_id not in self.backend.cameras:
                await websocket.send_json({
                    "type": "error",
                    "message": f"Camera {camera_id} not found"
                })
                await websocket.close()
                return

            self.backend.websocket_connections[camera_id].add(websocket)
            logger.info(f"WebSocket connected for camera {camera_id}")

            try:
                while True:
                    frame_buffer = self.backend.frame_buffers.get(camera_id)
                    if not frame_buffer:
                        await asyncio.sleep(0.05)
                        continue

                    frame = frame_buffer.get_latest_frame()
                    if frame is not None:
                        jpeg_b64 = base64.b64encode(frame.frame_data).decode()
                        await websocket.send_json({
                            "type": "frame",
                            "frame_data": jpeg_b64,
                            "timestamp": frame.timestamp
                        })

                    await asyncio.sleep(1 / 20)  # assuming 20 FPS target
            except WebSocketDisconnect:
                logger.info(f"WebSocket disconnected for camera {camera_id}")
            except Exception as e:
                logger.error(f"WebSocket error for {camera_id}: {e}")
            finally:
                self.backend.websocket_connections[camera_id].discard(websocket)

        @self.app.get("/api/health")
        async def health_check():
            return {
                "status": "healthy",
                "cameras_count": len(self.backend.cameras),
                "active_connections": sum(len(v) for v in self.backend.websocket_connections.values()),
                "timestamp": time.time()
            }
