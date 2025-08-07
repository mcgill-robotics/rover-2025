#!/usr/bin/env python3
"""
Central backend server for multi-camera streaming system.
Receives UDP frames from Jetson/Pi devices and serves them via WebSocket to frontend.
"""

import asyncio
import json
import socket
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

import json
import os

def get_backend_config():
    """Get backend configuration from service_config.yml."""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'service_config.yml')
    
    try:
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        camera_config = config['services']['camera_service']['config']
        
        # Convert to old format for compatibility
        return {
            "HOST": camera_config["host"],
            "HTTP_PORT": 8001,  # This will be overridden by service manager
            "UDP_PORT": 9999,
            "INACTIVE_TIMEOUT": camera_config["inactive_timeout"],
            "HEARTBEAT_TIMEOUT": 15.0,
            "FRAME_BUFFER_SIZE": camera_config["frame_buffer_size"],
            "MAX_UDP_PACKET_SIZE": 65507,
            "PORT_RANGE": camera_config["port_range"],
            "CAMERA_DISCOVERY": camera_config["camera_discovery"],
            "GSTREAMER_CONFIG": camera_config["gstreamer_config"],
            "ARUCO_CONFIG": camera_config["aruco_config"],
            "JPEG_CONFIG": camera_config["jpeg_config"]
        }
    except (FileNotFoundError, KeyError, yaml.YAMLError) as e:
        print(f"Warning: Could not load service config: {e}")
        # Return default config
        return {
            "HOST": "0.0.0.0",
            "HTTP_PORT": 8001,
            "UDP_PORT": 9999,
            "INACTIVE_TIMEOUT": 30.0,
            "HEARTBEAT_TIMEOUT": 15.0,
            "FRAME_BUFFER_SIZE": 10,
            "MAX_UDP_PACKET_SIZE": 65507,
            "CAMERA_DISCOVERY": {"enabled": True, "scan_interval": 5.0, "auto_connect": True},
            "GSTREAMER_CONFIG": {
                "rtp_caps": "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
                "pipeline_elements": ["rtph264depay", "h264parse", "avdec_h264", "videoconvert", "video/x-raw,format=BGR", "appsink drop=true max-buffers=2"],
                "buffer_size": 1,
                "drop_frames": True
            },
            "ARUCO_CONFIG": {
                "dictionary": "DICT_4X4_100",
                "marker_border_color": [0, 255, 0],
                "marker_border_thickness": 2,
                "text_color": [255, 255, 255],
                "text_thickness": 2,
                "text_font": "FONT_HERSHEY_SIMPLEX",
                "text_scale": 0.7
            },
            "JPEG_CONFIG": {
                "quality": 85,
                "optimize": True
            }
        }
from aruco_detector import create_aruco_detector
from gstreamer_reader import GStreamerCameraReader, MultiGStreamerReader

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
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

        # UDP socket for receiving heartbeats
        self.udp_sock = None
        self.udp_port = get_backend_config()["UDP_PORT"]

        # Track connected Jetson devices and their command ports
        self.jetson_devices: Dict[str, dict] = {}  # device_id -> {host, command_port, last_heartbeat}

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
            """Get currently active cameras."""
            current_time = time.time()
            active_cameras = []
            for camera_id, info in self.cameras.items():
                if current_time - info.last_frame_time < self.inactive_timeout:
                    active_cameras.append({**info.__dict__})
            return {"cameras": active_cameras}

        @self.app.get("/api/cameras/available")
        async def get_available_cameras():
            """Get all available cameras from connected Jetson devices."""
            available_cameras = []
            current_time = time.time()
            
            for device_id, device_info in self.jetson_devices.items():
                # Only include devices that have sent recent heartbeats
                if current_time - device_info['last_heartbeat'] < 30:
                    # Get camera info from the stored heartbeat data
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
                "total_devices": len(self.jetson_devices),
                "total_cameras": len(available_cameras)
            }

        @self.app.post("/api/cameras/{camera_id}/start")
        async def start_camera(camera_id: str):
            """Request to start a specific camera."""
            try:
                # Find which device this camera belongs to
                device_id = None
                for cam_id, cam_info in self.cameras.items():
                    if cam_id == camera_id:
                        device_id = cam_info.device_id
                        break
                
                # If camera not found in active cameras, try to find it from device info
                if not device_id:
                    # Extract device_id from camera_id (assuming format like "jetson-01-cam00")
                    parts = camera_id.split('-')
                    if len(parts) >= 2:
                        potential_device_id = '-'.join(parts[:-1])  # e.g., "jetson-01"
                        if potential_device_id in self.jetson_devices:
                            device_id = potential_device_id
                
                if not device_id:
                    return {"error": f"Cannot determine device for camera {camera_id}"}
                
                # Send start command to Jetson device
                command = {
                    "type": "start_camera",
                    "camera_id": camera_id
                }
                
                success = self.send_command_to_jetson(device_id, command)
                
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
                # Find which device this camera belongs to
                device_id = None
                if camera_id in self.cameras:
                    device_id = self.cameras[camera_id].device_id
                else:
                    # Extract device_id from camera_id (assuming format like "jetson-01-cam00")
                    parts = camera_id.split('-')
                    if len(parts) >= 2:
                        potential_device_id = '-'.join(parts[:-1])  # e.g., "jetson-01"
                        if potential_device_id in self.jetson_devices:
                            device_id = potential_device_id
                
                if not device_id:
                    return {"error": f"Cannot determine device for camera {camera_id}"}
                
                # Send stop command to Jetson device
                command = {
                    "type": "stop_camera",
                    "camera_id": camera_id
                }
                
                success = self.send_command_to_jetson(device_id, command)
                
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

            if camera_id not in self.cameras:
                await websocket.send_json({
                    "type": "error",
                    "message": f"Camera {camera_id} not found"
                })
                await websocket.close()
                return

            self.websocket_connections[camera_id].add(websocket)
            logger.info(f"WebSocket connected for camera {camera_id}")

            try:
                while True:
                    frame_buffer = self.frame_buffers.get(camera_id)
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
                self.websocket_connections[camera_id].discard(websocket)


        @self.app.get("/api/health")
        async def health_check():
            return {
                "status": "healthy",
                "cameras_count": len(self.cameras),
                "active_connections": sum(len(v) for v in self.websocket_connections.values()),
                "timestamp": time.time()
            }

    def setup_udp_socket(self):
        """Setup UDP socket for receiving heartbeats from Jetson devices."""
        try:
            import socket
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.bind(('0.0.0.0', self.udp_port))
            self.udp_sock.settimeout(1.0)  # 1 second timeout
            logger.info(f"UDP socket listening on port {self.udp_port} for heartbeats")
        except Exception as e:
            logger.error(f"Failed to setup UDP socket: {e}")
            self.udp_sock = None

    def parse_udp_packet(self, data: bytes) -> Optional[dict]:
        """Parse UDP packet from Jetson device."""
        try:
            import struct
            if len(data) < 13:  # Minimum header size
                return None
            
            # Parse header: [camera_id_len][packet_num][total_packets][timestamp]
            camera_id_len, packet_num, total_packets, timestamp = struct.unpack('!BHHQ', data[:13])
            
            if camera_id_len == 0 or camera_id_len > 100:  # Sanity check
                return None
            
            # Extract camera_id
            camera_id = data[13:13+camera_id_len].decode('utf-8')
            
            # Extract payload
            payload = data[13+camera_id_len:]
            
            return {
                'camera_id': camera_id,
                'packet_num': packet_num,
                'total_packets': total_packets,
                'timestamp': timestamp,
                'payload': payload
            }
            
        except Exception as e:
            logger.error(f"Failed to parse UDP packet: {e}")
            return None

    def send_command_to_jetson(self, device_id: str, command: dict):
        """Send command to Jetson device."""
        try:
            if device_id not in self.jetson_devices:
                logger.error(f"Jetson device {device_id} not found")
                return False
                
            device_info = self.jetson_devices[device_id]
            host = device_info['host']
            command_port = device_info['command_port']
            
            # Send command via UDP
            command_json = json.dumps(command).encode('utf-8')
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(command_json, (host, command_port))
            sock.close()
            
            logger.info(f"Sent command to {device_id} at {host}:{command_port}: {command}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send command to {device_id}: {e}")
            return False

    def handle_heartbeat(self, heartbeat_data: dict, sender_addr: tuple):
        """Handle heartbeat message from Jetson device."""
        try:
            device_id = heartbeat_data.get('device_id')
            cameras = heartbeat_data.get('cameras', {})
            
            # Update Jetson device info
            if device_id:
                self.jetson_devices[device_id] = {
                    'host': sender_addr[0],
                    'command_port': self.udp_port + 1,  # Jetson uses udp_port + 1 for commands
                    'last_heartbeat': time.time(),
                    'cameras': cameras  # Store camera information from heartbeat
                }
            
            logger.debug(f"Received heartbeat from {device_id} with {len(cameras)} cameras")
            
            # Only track available cameras, don't automatically start them
            for camera_id, camera_info in cameras.items():
                name = camera_info.get('name', camera_id)
                device_path = camera_info.get('device_path', '')
                is_active = camera_info.get('is_active', False)
                rtp_port = camera_info.get('rtp_port')
                
                # If camera is active and has RTP port, handle it
                if is_active and rtp_port:
                    if camera_id not in self.cameras:
                        # Add new active camera
                        try:
                            reader = self.camera_readers.add_camera(camera_id, rtp_port)
                            
                            self.cameras[camera_id] = CameraInfo(
                                camera_id=camera_id,
                                device_id=device_id,
                                name=name,
                                port=rtp_port,
                                is_active=True,
                                last_frame_time=time.time(),
                                resolution=reader.get_resolution(),
                                fps=reader.get_frame_rate()
                            )
                            
                            self.frame_buffers[camera_id] = FrameBuffer()
                            
                            # Start polling thread for this camera
                            thread = threading.Thread(
                                target=self.camera_polling_loop,
                                args=(camera_id, reader),
                                daemon=True
                            )
                            thread.start()
                            
                            logger.info(f"Added active camera {camera_id} from heartbeat on RTP port {rtp_port}")
                            
                        except Exception as e:
                            logger.error(f"Failed to add camera {camera_id} from heartbeat: {e}")
                    
                    else:
                        # Update existing camera
                        camera = self.cameras[camera_id]
                        if camera.port != rtp_port:
                            # Port changed, need to restart camera reader
                            logger.info(f"Camera {camera_id} port changed from {camera.port} to {rtp_port}")
                            
                            # Remove old reader
                            self.camera_readers.remove_camera(camera_id)
                            
                            # Add new reader with new port
                            try:
                                reader = self.camera_readers.add_camera(camera_id, rtp_port)
                                camera.port = rtp_port
                                camera.is_active = True
                                camera.last_frame_time = time.time()
                                
                                # Start new polling thread
                                thread = threading.Thread(
                                    target=self.camera_polling_loop,
                                    args=(camera_id, reader),
                                    daemon=True
                                )
                                thread.start()
                                
                                logger.info(f"Updated camera {camera_id} to new RTP port {rtp_port}")
                                
                            except Exception as e:
                                logger.error(f"Failed to update camera {camera_id} port: {e}")
                        
                        else:
                            # Just update timestamp
                            camera.last_frame_time = time.time()
                            camera.is_active = True
                
                elif camera_id in self.cameras and not is_active:
                    # Camera became inactive, remove it
                    logger.info(f"Removing inactive camera {camera_id}")
                    self.camera_readers.remove_camera(camera_id)
                    del self.cameras[camera_id]
                    self.frame_buffers.pop(camera_id, None)
                    
                    # Close WebSocket connections
                    connections = self.websocket_connections.pop(camera_id, set())
                    for ws in connections:
                        asyncio.run_coroutine_threadsafe(ws.close(), self.loop)
                    
        except Exception as e:
            logger.error(f"Failed to handle heartbeat: {e}")

    def udp_receiver_thread(self):
        """Thread to receive UDP packets from Jetson devices."""
        import socket
        logger.info("Starting UDP receiver thread")
        
        while self.running:
            if not self.udp_sock:
                time.sleep(1)
                continue
                
            try:
                data, addr = self.udp_sock.recvfrom(65536)
                logger.debug(f"Received UDP packet from {addr}, size: {len(data)} bytes")
                
                packet_info = self.parse_udp_packet(data)
                
                if packet_info:
                    camera_id = packet_info['camera_id']
                    logger.debug(f"Parsed packet with camera_id: {camera_id}")
                    
                    if camera_id == '__HEARTBEAT__':
                        # Handle heartbeat
                        try:
                            heartbeat_data = json.loads(packet_info['payload'].decode('utf-8'))
                            logger.info(f"Received heartbeat from {heartbeat_data.get('device_id', 'unknown')} at {addr}")
                            self.handle_heartbeat(heartbeat_data, addr)
                        except Exception as e:
                            logger.error(f"Failed to parse heartbeat JSON: {e}")
                    
                    # Note: We no longer handle frame data here since we use GStreamer RTP streams
                else:
                    logger.warning(f"Failed to parse UDP packet from {addr}")
                    
            except socket.timeout:
                # Normal timeout, continue
                continue
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    logger.debug(f"UDP receive error: {e}")
                time.sleep(0.1)
        
        logger.info("UDP receiver thread ended")

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
        
        # Setup UDP socket for heartbeats
        self.setup_udp_socket()
        
        # Start UDP receiver thread
        threading.Thread(target=self.udp_receiver_thread, daemon=True).start()
        
        # Wait a moment for heartbeats to discover cameras
        logger.info("Waiting for heartbeats from Jetson devices...")
        time.sleep(3) 
        
        # Start camera polling threads for any existing cameras
        self.start_camera_polling_threads()
        
        # Start cleanup thread
        threading.Thread(target=self.cleanup_thread, daemon=True).start()
        
        # Start the web server
        try:
            uvicorn.run(self.app, host="0.0.0.0", port=self.http_port, log_level="info")
        finally:
            self.running = False
            self.camera_readers.release_all()
            if self.udp_sock:
                self.udp_sock.close()


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
