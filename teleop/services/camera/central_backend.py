#!/usr/bin/env python3
"""
Central backend server for multi-camera streaming system.
Receives UDP frames from Jetson/Pi devices and serves them via WebSocket to frontend.
"""

import asyncio
import json
import socket
import struct
import time
import threading
from collections import defaultdict, deque
from typing import Dict, List, Optional, Set
import logging
from dataclasses import dataclass, asdict
import base64

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from config import get_backend_config, get_logging_config
from aruco_detector import create_aruco_detector

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
    device_path: str
    is_active: bool
    last_frame_time: float
    last_heartbeat_time: float

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
    def __init__(self, udp_port: int = 9999, http_port: int = 8001, enable_aruco: bool = True):
        self.udp_port = udp_port
        self.http_port = http_port
        self.enable_aruco = enable_aruco
        
        # Camera management
        self.cameras: Dict[str, CameraInfo] = {}
        self.frame_buffers: Dict[str, FrameBuffer] = {}
        self.partial_frames: Dict[str, Dict[int, bytes]] = defaultdict(dict)
        
        # WebSocket connections
        self.websocket_connections: Dict[str, Set[WebSocket]] = defaultdict(set)
        
        # UDP socket
        self.udp_socket = None
        self.running = False
        
        # Cleanup settings
        self.inactive_timeout = 30.0  # seconds
        self.heartbeat_timeout = 15.0  # seconds
        
        # ArUco detection
        self.aruco_detector = None
        if self.enable_aruco:
            try:
                self.aruco_detector = create_aruco_detector("DICT_4X4_100")
                logger.info("ArUco detector initialized")
            except Exception as e:
                logger.error(f"Failed to initialize ArUco detector: {e}")
                self.enable_aruco = False
        
        # FastAPI app
        self.app = FastAPI(title="Multi-Camera Backend", version="1.0.0")
        self.setup_routes()
        self.setup_cors()
    
    def setup_cors(self):
        """Setup CORS middleware."""
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
    
    def setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/api/cameras")
        async def get_cameras():
            """Get list of active cameras."""
            current_time = time.time()
            active_cameras = []
            
            for camera_id, camera_info in self.cameras.items():
                # Check if camera is still active based on recent frames or heartbeat
                is_recently_active = (
                    current_time - camera_info.last_frame_time < self.inactive_timeout or
                    current_time - camera_info.last_heartbeat_time < self.heartbeat_timeout
                )
                
                if is_recently_active:
                    active_cameras.append({
                        "camera_id": camera_id,
                        "device_id": camera_info.device_id,
                        "name": camera_info.name,
                        "device_path": camera_info.device_path,
                        "is_active": camera_info.is_active,
                        "last_frame_time": camera_info.last_frame_time,
                        "last_heartbeat_time": camera_info.last_heartbeat_time
                    })
            
            return {"cameras": active_cameras}
        
        @self.app.websocket("/stream")
        async def websocket_stream(websocket: WebSocket, camera_id: str):
            """WebSocket endpoint for streaming camera frames."""
            await websocket.accept()
            
            if camera_id not in self.cameras:
                await websocket.send_json({
                    "type": "error",
                    "message": f"Camera {camera_id} not found"
                })
                await websocket.close()
                return
            
            # Add connection to the camera's connection set
            self.websocket_connections[camera_id].add(websocket)
            logger.info(f"WebSocket connected for camera {camera_id}")
            
            try:
                # Send initial status
                await websocket.send_json({
                    "type": "status",
                    "camera_id": camera_id,
                    "message": "Connected"
                })
                
                # Keep connection alive and handle incoming messages
                while True:
                    try:
                        # Wait for messages from client (ping/pong, etc.)
                        message = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                        # Handle client messages if needed
                        data = json.loads(message)
                        if data.get("type") == "ping":
                            await websocket.send_json({"type": "pong"})
                    except asyncio.TimeoutError:
                        # No message received, continue
                        pass
                    except WebSocketDisconnect:
                        break
                    
            except WebSocketDisconnect:
                pass
            except Exception as e:
                logger.error(f"WebSocket error for camera {camera_id}: {e}")
            finally:
                # Remove connection
                self.websocket_connections[camera_id].discard(websocket)
                logger.info(f"WebSocket disconnected for camera {camera_id}")
        
        @self.app.get("/api/health")
        async def health_check():
            """Health check endpoint."""
            return {
                "status": "healthy",
                "cameras_count": len(self.cameras),
                "active_connections": sum(len(conns) for conns in self.websocket_connections.values()),
                "timestamp": time.time()
            }
    
    def setup_udp_socket(self):
        """Setup UDP socket for receiving frames."""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB buffer
        self.udp_socket.bind(('0.0.0.0', self.udp_port))
        logger.info(f"UDP socket listening on port {self.udp_port}")
    
    def parse_udp_packet(self, data: bytes) -> Optional[tuple]:
        """Parse incoming UDP packet."""
        try:
            if len(data) < 13:  # Minimum header size
                return None
            
            # Parse header: [camera_id_len][packet_num][total_packets][timestamp]
            camera_id_len, packet_num, total_packets, timestamp = struct.unpack('!BHHQ', data[:13])
            
            if len(data) < 13 + camera_id_len:
                return None
            
            # Extract camera ID
            camera_id = data[13:13 + camera_id_len].decode('utf-8')
            
            # Extract frame data
            frame_data = data[13 + camera_id_len:]
            
            return camera_id, packet_num, total_packets, timestamp, frame_data
            
        except Exception as e:
            logger.error(f"Failed to parse UDP packet: {e}")
            return None
    
    def handle_heartbeat(self, camera_id: str, heartbeat_data: bytes):
        """Handle heartbeat packet from device."""
        try:
            heartbeat_json = json.loads(heartbeat_data.decode('utf-8'))
            device_id = heartbeat_json.get('device_id', 'unknown')
            cameras_status = heartbeat_json.get('cameras', {})
            
            current_time = time.time()
            
            # Update camera information from heartbeat
            for cam_id, cam_status in cameras_status.items():
                if cam_id not in self.cameras:
                    # New camera discovered
                    self.cameras[cam_id] = CameraInfo(
                        camera_id=cam_id,
                        device_id=device_id,
                        name=cam_status.get('name', 'Unknown Camera'),
                        device_path=cam_status.get('device_path', ''),
                        is_active=cam_status.get('is_active', False),
                        last_frame_time=cam_status.get('last_frame_time', 0),
                        last_heartbeat_time=current_time
                    )
                    self.frame_buffers[cam_id] = FrameBuffer()
                    logger.info(f"New camera registered: {cam_id} from device {device_id}")
                else:
                    # Update existing camera
                    camera_info = self.cameras[cam_id]
                    camera_info.is_active = cam_status.get('is_active', False)
                    camera_info.last_heartbeat_time = current_time
                    if cam_status.get('last_frame_time', 0) > camera_info.last_frame_time:
                        camera_info.last_frame_time = cam_status.get('last_frame_time', 0)
            
        except Exception as e:
            logger.error(f"Failed to handle heartbeat: {e}")
    
    def handle_frame_packet(self, camera_id: str, packet_num: int, total_packets: int, 
                           timestamp: int, frame_data: bytes):
        """Handle frame packet from device."""
        if total_packets == 1:
            # Single packet frame
            self.process_complete_frame(camera_id, timestamp, frame_data)
        else:
            # Multi-packet frame
            frame_key = f"{camera_id}_{timestamp}"
            
            # Store packet
            if frame_key not in self.partial_frames:
                self.partial_frames[frame_key] = {}
            
            self.partial_frames[frame_key][packet_num] = frame_data
            
            # Check if we have all packets
            if len(self.partial_frames[frame_key]) == total_packets:
                # Reconstruct frame
                complete_frame = b''
                for i in range(total_packets):
                    if i in self.partial_frames[frame_key]:
                        complete_frame += self.partial_frames[frame_key][i]
                
                # Clean up partial frame data
                del self.partial_frames[frame_key]
                
                # Process complete frame
                self.process_complete_frame(camera_id, timestamp, complete_frame)
    
    def process_complete_frame(self, camera_id: str, timestamp: int, frame_data: bytes):
        """Process a complete frame."""
        current_time = time.time()
        
        # Update camera info
        if camera_id in self.cameras:
            self.cameras[camera_id].last_frame_time = current_time
        
        # Store frame in buffer
        if camera_id in self.frame_buffers:
            frame = CameraFrame(
                camera_id=camera_id,
                timestamp=timestamp,
                frame_data=frame_data,
                received_time=current_time
            )
            self.frame_buffers[camera_id].add_frame(frame)
            
            # Send frame to WebSocket connections
            asyncio.create_task(self.broadcast_frame(camera_id, frame))
    
    async def broadcast_frame(self, camera_id: str, frame: CameraFrame):
        """Broadcast frame to all WebSocket connections for this camera."""
        if camera_id not in self.websocket_connections:
            return
        
        connections = self.websocket_connections[camera_id].copy()
        if not connections:
            return
        
        # Process frame with ArUco detection if enabled
        processed_frame_data = frame.frame_data
        aruco_detected = False
        
        if self.enable_aruco and self.aruco_detector:
            try:
                # Process frame for ArUco markers
                processed_data = self.aruco_detector.process_frame(frame.frame_data, is_h264=True)
                if processed_data and processed_data != frame.frame_data:
                    processed_frame_data = processed_data
                    aruco_detected = True
                    logger.debug(f"ArUco processing applied to frame from {camera_id}")
            except Exception as e:
                logger.error(f"ArUco processing failed for {camera_id}: {e}")
                # Fall back to original frame data
                processed_frame_data = frame.frame_data
        
        # Encode frame data as base64 for JSON transmission
        frame_b64 = base64.b64encode(processed_frame_data).decode('utf-8')
        
        message = {
            "type": "frame",
            "camera_id": camera_id,
            "timestamp": frame.timestamp,
            "received_time": frame.received_time,
            "frame_data": frame_b64,
            "aruco_detected": aruco_detected
        }
        
        # Send to all connections
        disconnected = set()
        for websocket in connections:
            try:
                await websocket.send_json(message)
            except Exception as e:
                logger.warning(f"Failed to send frame to WebSocket: {e}")
                disconnected.add(websocket)
        
        # Remove disconnected WebSockets
        for websocket in disconnected:
            self.websocket_connections[camera_id].discard(websocket)
    
    def udp_receiver_thread(self):
        """UDP receiver thread."""
        logger.info("Starting UDP receiver thread")
        
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(65536)
                
                parsed = self.parse_udp_packet(data)
                if not parsed:
                    continue
                
                camera_id, packet_num, total_packets, timestamp, frame_data = parsed
                
                # Handle heartbeat packets
                if camera_id == '__HEARTBEAT__':
                    self.handle_heartbeat(camera_id, frame_data)
                else:
                    # Handle frame packets
                    self.handle_frame_packet(camera_id, packet_num, total_packets, 
                                           timestamp, frame_data)
                
            except Exception as e:
                if self.running:
                    logger.error(f"UDP receiver error: {e}")
                    time.sleep(0.1)
    
    def cleanup_thread(self):
        """Cleanup thread for inactive cameras and old data."""
        logger.info("Starting cleanup thread")
        
        while self.running:
            try:
                current_time = time.time()
                
                # Clean up inactive cameras
                inactive_cameras = []
                for camera_id, camera_info in self.cameras.items():
                    if (current_time - camera_info.last_frame_time > self.inactive_timeout and
                        current_time - camera_info.last_heartbeat_time > self.heartbeat_timeout):
                        inactive_cameras.append(camera_id)
                
                for camera_id in inactive_cameras:
                    logger.info(f"Removing inactive camera: {camera_id}")
                    del self.cameras[camera_id]
                    if camera_id in self.frame_buffers:
                        del self.frame_buffers[camera_id]
                    if camera_id in self.websocket_connections:
                        # Close all WebSocket connections for this camera
                        connections = self.websocket_connections[camera_id].copy()
                        for websocket in connections:
                            try:
                                asyncio.create_task(websocket.close())
                            except:
                                pass
                        del self.websocket_connections[camera_id]
                
                # Clean up old partial frames (older than 5 seconds)
                old_frames = []
                for frame_key in self.partial_frames:
                    try:
                        timestamp_str = frame_key.split('_')[-1]
                        timestamp = int(timestamp_str)
                        if current_time * 1000 - timestamp > 5000:  # 5 seconds
                            old_frames.append(frame_key)
                    except:
                        old_frames.append(frame_key)
                
                for frame_key in old_frames:
                    del self.partial_frames[frame_key]
                
                time.sleep(10)  # Run cleanup every 10 seconds
                
            except Exception as e:
                logger.error(f"Cleanup thread error: {e}")
                time.sleep(10)
    
    def run(self):
        """Run the backend server."""
        logger.info(f"Starting multi-camera backend server")
        logger.info(f"UDP port: {self.udp_port}, HTTP port: {self.http_port}")
        
        self.running = True
        
        # Setup UDP socket
        self.setup_udp_socket()
        
        # Start UDP receiver thread
        udp_thread = threading.Thread(target=self.udp_receiver_thread)
        udp_thread.daemon = True
        udp_thread.start()
        
        # Start cleanup thread
        cleanup_thread = threading.Thread(target=self.cleanup_thread)
        cleanup_thread.daemon = True
        cleanup_thread.start()
        
        try:
            # Run FastAPI server
            uvicorn.run(
                self.app,
                host="0.0.0.0",
                port=self.http_port,
                log_level="info"
            )
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.running = False
            if self.udp_socket:
                self.udp_socket.close()

def main():
    import argparse
    
    # Load config defaults
    config = get_backend_config()
    
    parser = argparse.ArgumentParser(description='Multi-camera central backend server')
    parser.add_argument('--udp-port', type=int, default=config["UDP_PORT"], 
                       help='UDP port for receiving frames')
    parser.add_argument('--http-port', type=int, default=config["HTTP_PORT"], 
                       help='HTTP port for API and WebSocket')
    parser.add_argument('--inactive-timeout', type=float, default=config["INACTIVE_TIMEOUT"],
                       help='Timeout for inactive cameras (seconds)')
    parser.add_argument('--heartbeat-timeout', type=float, default=config["HEARTBEAT_TIMEOUT"],
                       help='Timeout for heartbeat (seconds)')
    parser.add_argument('--enable-aruco', action='store_true', default=True,
                       help='Enable ArUco marker detection (default: True)')
    parser.add_argument('--disable-aruco', action='store_true', default=False,
                       help='Disable ArUco marker detection')
    
    args = parser.parse_args()
    
    # Handle ArUco enable/disable flags
    enable_aruco = args.enable_aruco and not args.disable_aruco
    
    backend = MultiCameraBackend(
        udp_port=args.udp_port,
        http_port=args.http_port,
        enable_aruco=enable_aruco
    )
    
    backend.inactive_timeout = args.inactive_timeout
    backend.heartbeat_timeout = args.heartbeat_timeout
    
    backend.run()

if __name__ == "__main__":
    main()
