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
from dataclasses import dataclass
import base64

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from config import get_backend_config
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
    def __init__(self, udp_port: int = 9999, http_port: int = 8001, enable_aruco: bool = True, loop: Optional[asyncio.AbstractEventLoop] = None):
        self.udp_port = udp_port
        self.http_port = http_port
        self.enable_aruco = enable_aruco
        self.loop = loop or asyncio.get_event_loop()

        self.cameras: Dict[str, CameraInfo] = {}
        self.frame_buffers: Dict[str, FrameBuffer] = {}
        self.partial_frames: Dict[str, Dict[int, bytes]] = defaultdict(dict)
        self.websocket_connections: Dict[str, Set[WebSocket]] = defaultdict(set)

        self.udp_socket = None
        self.running = False

        self.inactive_timeout = 30.0
        self.heartbeat_timeout = 15.0

        self.aruco_detector = None
        if self.enable_aruco:
            try:
                self.aruco_detector = create_aruco_detector("DICT_4X4_100")
                logger.info("ArUco detector initialized")
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
                if (current_time - info.last_frame_time < self.inactive_timeout or
                    current_time - info.last_heartbeat_time < self.heartbeat_timeout):
                    active_cameras.append({**info.__dict__})
            return {"cameras": active_cameras}

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

    def setup_udp_socket(self):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.udp_socket.bind(('0.0.0.0', self.udp_port))
        logger.info(f"UDP socket listening on port {self.udp_port}")

    def parse_udp_packet(self, data: bytes) -> Optional[tuple]:
        try:
            if len(data) < 13:
                return None
            camera_id_len, packet_num, total_packets, timestamp = struct.unpack('!BHHQ', data[:13])
            if len(data) < 13 + camera_id_len:
                return None
            camera_id = data[13:13 + camera_id_len].decode('utf-8')
            frame_data = data[13 + camera_id_len:]
            return camera_id, packet_num, total_packets, timestamp, frame_data
        except Exception as e:
            logger.error(f"Failed to parse UDP packet: {e}")
            return None

    def handle_heartbeat(self, camera_id: str, heartbeat_data: bytes):
        try:
            heartbeat_json = json.loads(heartbeat_data.decode('utf-8'))
            device_id = heartbeat_json.get('device_id', 'unknown')
            cameras_status = heartbeat_json.get('cameras', {})
            now = time.time()
            for cam_id, status in cameras_status.items():
                if cam_id not in self.cameras:
                    self.cameras[cam_id] = CameraInfo(cam_id, device_id, status.get('name', 'Unknown'), status.get('device_path', ''), status.get('is_active', False), status.get('last_frame_time', 0), now)
                    self.frame_buffers[cam_id] = FrameBuffer()
                    logger.info(f"New camera registered: {cam_id} from device {device_id}")
                else:
                    info = self.cameras[cam_id]
                    info.is_active = status.get('is_active', False)
                    info.last_heartbeat_time = now
                    if status.get('last_frame_time', 0) > info.last_frame_time:
                        info.last_frame_time = status.get('last_frame_time', 0)
        except Exception as e:
            logger.error(f"Failed to handle heartbeat: {e}")

    def handle_frame_packet(self, camera_id: str, packet_num: int, total_packets: int, timestamp: int, frame_data: bytes):
        if total_packets == 1:
            self.process_complete_frame(camera_id, timestamp, frame_data)
        else:
            key = f"{camera_id}_{timestamp}"
            self.partial_frames[key][packet_num] = frame_data
            if len(self.partial_frames[key]) == total_packets:
                full_frame = b''.join(self.partial_frames[key][i] for i in range(total_packets))
                del self.partial_frames[key]
                self.process_complete_frame(camera_id, timestamp, full_frame)

    def process_complete_frame(self, camera_id: str, timestamp: int, frame_data: bytes):
        now = time.time()
        if camera_id in self.cameras:
            self.cameras[camera_id].last_frame_time = now
        if camera_id in self.frame_buffers:
            frame = CameraFrame(camera_id, timestamp, frame_data, now)
            self.frame_buffers[camera_id].add_frame(frame)
            try:
                asyncio.run_coroutine_threadsafe(self.broadcast_frame(camera_id, frame), self.loop)
            except Exception as e:
                logger.error(f"Failed to schedule broadcast: {e}")

    async def broadcast_frame(self, camera_id: str, frame: CameraFrame):
        connections = self.websocket_connections.get(camera_id, set()).copy()
        if not connections:
            return
        processed_data = frame.frame_data
        aruco_detected = False
        if self.enable_aruco and self.aruco_detector:
            try:
                result = self.aruco_detector.process_frame(frame.frame_data, is_h264=True)
                if result:
                    processed_data = result
                    aruco_detected = True
            except Exception as e:
                logger.error(f"ArUco detection failed: {e}")
        encoded = base64.b64encode(processed_data).decode('utf-8')
        message = {
            "type": "frame",
            "camera_id": camera_id,
            "timestamp": frame.timestamp,
            "received_time": frame.received_time,
            "frame_data": encoded,
            "aruco_detected": aruco_detected
        }
        for ws in connections:
            try:
                await ws.send_json(message)
            except Exception as e:
                logger.warning(f"WebSocket send failed: {e}")
                self.websocket_connections[camera_id].discard(ws)

    def udp_receiver_thread(self):
        logger.info("Starting UDP receiver thread")
        while self.running:
            try:
                data, _ = self.udp_socket.recvfrom(65536)
                parsed = self.parse_udp_packet(data)
                if parsed:
                    camera_id, packet_num, total_packets, timestamp, frame_data = parsed
                    if camera_id == '__HEARTBEAT__':
                        self.handle_heartbeat(camera_id, frame_data)
                    else:
                        self.handle_frame_packet(camera_id, packet_num, total_packets, timestamp, frame_data)
            except Exception as e:
                logger.error(f"UDP receiver error: {e}")
                time.sleep(0.1)

    def cleanup_thread(self):
        logger.info("Starting cleanup thread")
        while self.running:
            try:
                now = time.time()
                for cam_id in list(self.cameras):
                    info = self.cameras[cam_id]
                    if now - info.last_frame_time > self.inactive_timeout and now - info.last_heartbeat_time > self.heartbeat_timeout:
                        logger.info(f"Removing inactive camera: {cam_id}")
                        del self.cameras[cam_id]
                        self.frame_buffers.pop(cam_id, None)
                        conns = self.websocket_connections.pop(cam_id, set())
                        for ws in conns:
                            asyncio.run_coroutine_threadsafe(ws.close(), self.loop)
                time.sleep(10)
            except Exception as e:
                logger.error(f"Cleanup error: {e}")
                time.sleep(10)

    def run(self):
        logger.info("Starting multi-camera backend server")
        self.running = True
        self.setup_udp_socket()
        threading.Thread(target=self.udp_receiver_thread, daemon=True).start()
        threading.Thread(target=self.cleanup_thread, daemon=True).start()
        uvicorn.run(self.app, host="0.0.0.0", port=self.http_port, log_level="info")


def main():
    import argparse
    config = get_backend_config()
    parser = argparse.ArgumentParser()
    parser.add_argument('--udp-port', type=int, default=config["UDP_PORT"])
    parser.add_argument('--http-port', type=int, default=config["HTTP_PORT"])
    parser.add_argument('--inactive-timeout', type=float, default=config["INACTIVE_TIMEOUT"])
    parser.add_argument('--heartbeat-timeout', type=float, default=config["HEARTBEAT_TIMEOUT"])
    parser.add_argument('--enable-aruco', action='store_true', default=True)
    parser.add_argument('--disable-aruco', action='store_true', default=False)
    args = parser.parse_args()

    enable_aruco = args.enable_aruco and not args.disable_aruco

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    backend = MultiCameraBackend(
        udp_port=args.udp_port,
        http_port=args.http_port,
        enable_aruco=enable_aruco,
        loop=loop
    )
    backend.inactive_timeout = args.inactive_timeout
    backend.heartbeat_timeout = args.heartbeat_timeout
    backend.run()


if __name__ == "__main__":
    main()
