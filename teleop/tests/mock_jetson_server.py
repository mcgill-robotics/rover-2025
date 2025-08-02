#!/usr/bin/env python3
"""
Mock Jetson Server for Testing Camera System

Simulates multiple Jetson devices, each with multiple cameras using video files
instead of hardware cameras. This allows testing the complete camera management
system without physical Jetson devices.

Usage:
    python3 mock_jetson_server.py

Features:
- Simulates 3 Jetson devices (jetson-01, jetson-02, jetson-03)
- Each device has 2-3 cameras using generated video patterns
- Responds to start/stop commands from central_backend
- Sends heartbeats and device discovery
- Streams video data when requested
"""

import asyncio
import websockets
import json
import time
import threading
import cv2
import numpy as np
import base64
import logging
from typing import Dict, List, Optional, Any
import argparse
from dataclasses import dataclass, asdict
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class CameraInfo:
    """Information about a simulated camera."""
    camera_id: str
    name: str
    resolution: tuple
    fps: int
    is_active: bool = False
    video_pattern: str = "color_bars"  # color_bars, checkerboard, gradient, noise

@dataclass
class JetsonDevice:
    """Information about a simulated Jetson device."""
    device_id: str
    name: str
    ip_address: str
    port: int
    cameras: List[CameraInfo]
    is_online: bool = True
    last_heartbeat: float = 0

class VideoPatternGenerator:
    """Generates test video patterns for simulated cameras."""
    
    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height
        self.frame_count = 0
    
    def generate_color_bars(self) -> np.ndarray:
        """Generate color bars test pattern."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        bar_width = self.width // 8
        
        colors = [
            (255, 255, 255),  # White
            (255, 255, 0),    # Yellow
            (0, 255, 255),    # Cyan
            (0, 255, 0),      # Green
            (255, 0, 255),    # Magenta
            (255, 0, 0),      # Red
            (0, 0, 255),      # Blue
            (0, 0, 0),        # Black
        ]
        
        for i, color in enumerate(colors):
            x_start = i * bar_width
            x_end = min((i + 1) * bar_width, self.width)
            frame[:, x_start:x_end] = color
        
        # Add frame counter
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return frame
    
    def generate_checkerboard(self) -> np.ndarray:
        """Generate checkerboard test pattern."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        square_size = 40
        
        for y in range(0, self.height, square_size):
            for x in range(0, self.width, square_size):
                if ((x // square_size) + (y // square_size)) % 2 == 0:
                    color = (255, 255, 255)
                else:
                    color = (0, 0, 0)
                
                frame[y:y+square_size, x:x+square_size] = color
        
        # Add moving element
        center_x = int(self.width/2 + 100 * np.sin(self.frame_count * 0.1))
        center_y = int(self.height/2 + 50 * np.cos(self.frame_count * 0.1))
        cv2.circle(frame, (center_x, center_y), 20, (0, 255, 0), -1)
        
        return frame
    
    def generate_gradient(self) -> np.ndarray:
        """Generate gradient test pattern."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create horizontal gradient
        for x in range(self.width):
            intensity = int(255 * x / self.width)
            frame[:, x] = (intensity, intensity // 2, 255 - intensity)
        
        # Add animated overlay
        overlay_y = int(self.height/2 + 50 * np.sin(self.frame_count * 0.05))
        cv2.rectangle(frame, (0, overlay_y-10), (self.width, overlay_y+10), 
                     (255, 255, 255), 2)
        
        return frame
    
    def generate_noise(self) -> np.ndarray:
        """Generate noise test pattern."""
        # Base noise
        frame = np.random.randint(0, 256, (self.height, self.width, 3), dtype=np.uint8)
        
        # Add some structure
        cv2.rectangle(frame, (50, 50), (self.width-50, self.height-50), 
                     (128, 128, 128), 3)
        cv2.putText(frame, f"NOISE {self.frame_count}", (60, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return frame
    
    def get_frame(self, pattern: str) -> np.ndarray:
        """Get a frame with the specified pattern."""
        self.frame_count += 1
        
        if pattern == "color_bars":
            return self.generate_color_bars()
        elif pattern == "checkerboard":
            return self.generate_checkerboard()
        elif pattern == "gradient":
            return self.generate_gradient()
        elif pattern == "noise":
            return self.generate_noise()
        else:
            return self.generate_color_bars()

class MockJetsonServer:
    """Mock Jetson server that simulates multiple devices."""
    
    def __init__(self, central_backend_url: str = "ws://localhost:8081/jetson"):
        self.central_backend_url = central_backend_url
        self.devices = self._create_mock_devices()
        self.video_generators = {}
        self.streaming_tasks = {}
        self.websocket = None
        self.running = False
        self.heartbeat_task = None
        
        # Initialize video generators for each camera
        for device in self.devices.values():
            for camera in device.cameras:
                key = f"{device.device_id}_{camera.camera_id}"
                self.video_generators[key] = VideoPatternGenerator(
                    camera.resolution[0], camera.resolution[1]
                )
    
    def _create_mock_devices(self) -> Dict[str, JetsonDevice]:
        """Create mock Jetson devices with cameras."""
        devices = {}
        
        # Jetson 01 - Front cameras
        devices["jetson-01"] = JetsonDevice(
            device_id="jetson-01",
            name="Front Camera Unit",
            ip_address="192.168.1.101",
            port=8080,
            cameras=[
                CameraInfo("cam_0", "Front Left", (640, 480), 30, pattern="color_bars"),
                CameraInfo("cam_1", "Front Right", (640, 480), 30, pattern="checkerboard"),
                CameraInfo("cam_2", "Front Center", (1280, 720), 30, pattern="gradient")
            ]
        )
        
        # Jetson 02 - Side cameras
        devices["jetson-02"] = JetsonDevice(
            device_id="jetson-02",
            name="Side Camera Unit",
            ip_address="192.168.1.102",
            port=8080,
            cameras=[
                CameraInfo("cam_0", "Left Side", (640, 480), 30, pattern="noise"),
                CameraInfo("cam_1", "Right Side", (640, 480), 30, pattern="color_bars")
            ]
        )
        
        # Jetson 03 - Rear cameras
        devices["jetson-03"] = JetsonDevice(
            device_id="jetson-03",
            name="Rear Camera Unit",
            ip_address="192.168.1.103",
            port=8080,
            cameras=[
                CameraInfo("cam_0", "Rear View", (1280, 720), 30, pattern="checkerboard"),
                CameraInfo("cam_1", "Rear Wide", (640, 480), 15, pattern="gradient"),
                CameraInfo("cam_2", "Backup Camera", (320, 240), 15, pattern="noise")
            ]
        )
        
        return devices
    
    async def connect_to_central_backend(self):
        """Connect to the central backend."""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                logger.info(f"Connecting to central backend: {self.central_backend_url}")
                self.websocket = await websockets.connect(self.central_backend_url)
                logger.info("Connected to central backend")
                return True
            except Exception as e:
                logger.warning(f"Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    await asyncio.sleep(retry_delay)
                    retry_delay *= 2
        
        logger.error("Failed to connect to central backend after all retries")
        return False
    
    async def send_device_discovery(self):
        """Send device discovery messages for all mock devices."""
        if not self.websocket:
            return
        
        for device in self.devices.values():
            discovery_msg = {
                "type": "device_discovery",
                "device_id": device.device_id,
                "device_info": {
                    "name": device.name,
                    "ip_address": device.ip_address,
                    "port": device.port,
                    "cameras": [
                        {
                            "camera_id": cam.camera_id,
                            "name": cam.name,
                            "resolution": cam.resolution,
                            "fps": cam.fps,
                            "is_active": cam.is_active
                        }
                        for cam in device.cameras
                    ]
                },
                "timestamp": time.time()
            }
            
            try:
                await self.websocket.send(json.dumps(discovery_msg))
                logger.info(f"Sent discovery for device: {device.device_id}")
            except Exception as e:
                logger.error(f"Failed to send discovery for {device.device_id}: {e}")
    
    async def send_heartbeat(self):
        """Send heartbeat messages for all devices."""
        while self.running and self.websocket:
            try:
                for device in self.devices.values():
                    if device.is_online:
                        heartbeat_msg = {
                            "type": "heartbeat",
                            "device_id": device.device_id,
                            "status": "online",
                            "active_cameras": [
                                cam.camera_id for cam in device.cameras if cam.is_active
                            ],
                            "timestamp": time.time()
                        }
                        
                        await self.websocket.send(json.dumps(heartbeat_msg))
                        device.last_heartbeat = time.time()
                
                await asyncio.sleep(5)  # Send heartbeat every 5 seconds
                
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                break
    
    async def handle_camera_command(self, message: dict):
        """Handle camera start/stop commands."""
        device_id = message.get("device_id")
        camera_id = message.get("camera_id")
        command = message.get("command")
        
        if device_id not in self.devices:
            logger.warning(f"Unknown device: {device_id}")
            return
        
        device = self.devices[device_id]
        camera = None
        
        for cam in device.cameras:
            if cam.camera_id == camera_id:
                camera = cam
                break
        
        if not camera:
            logger.warning(f"Unknown camera: {camera_id} on device {device_id}")
            return
        
        if command == "start":
            if not camera.is_active:
                camera.is_active = True
                await self.start_camera_stream(device_id, camera_id)
                logger.info(f"Started camera {camera_id} on device {device_id}")
        
        elif command == "stop":
            if camera.is_active:
                camera.is_active = False
                await self.stop_camera_stream(device_id, camera_id)
                logger.info(f"Stopped camera {camera_id} on device {device_id}")
        
        # Send acknowledgment
        ack_msg = {
            "type": "camera_command_ack",
            "device_id": device_id,
            "camera_id": camera_id,
            "command": command,
            "status": "success",
            "timestamp": time.time()
        }
        
        try:
            await self.websocket.send(json.dumps(ack_msg))
        except Exception as e:
            logger.error(f"Failed to send acknowledgment: {e}")
    
    async def start_camera_stream(self, device_id: str, camera_id: str):
        """Start streaming for a specific camera."""
        key = f"{device_id}_{camera_id}"
        
        if key in self.streaming_tasks:
            return  # Already streaming
        
        device = self.devices[device_id]
        camera = None
        
        for cam in device.cameras:
            if cam.camera_id == camera_id:
                camera = cam
                break
        
        if not camera:
            return
        
        # Start streaming task
        task = asyncio.create_task(
            self.stream_camera_data(device_id, camera_id, camera)
        )
        self.streaming_tasks[key] = task
    
    async def stop_camera_stream(self, device_id: str, camera_id: str):
        """Stop streaming for a specific camera."""
        key = f"{device_id}_{camera_id}"
        
        if key in self.streaming_tasks:
            self.streaming_tasks[key].cancel()
            del self.streaming_tasks[key]
    
    async def stream_camera_data(self, device_id: str, camera_id: str, camera: CameraInfo):
        """Stream video data for a camera."""
        key = f"{device_id}_{camera_id}"
        generator = self.video_generators[key]
        
        frame_interval = 1.0 / camera.fps
        
        try:
            while camera.is_active and self.running:
                # Generate frame
                frame = generator.get_frame(camera.video_pattern)
                
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_data = base64.b64encode(buffer).decode('utf-8')
                
                # Send frame data
                stream_msg = {
                    "type": "camera_frame",
                    "device_id": device_id,
                    "camera_id": camera_id,
                    "frame_data": frame_data,
                    "timestamp": time.time(),
                    "frame_number": generator.frame_count
                }
                
                if self.websocket:
                    try:
                        await self.websocket.send(json.dumps(stream_msg))
                    except Exception as e:
                        logger.error(f"Failed to send frame for {key}: {e}")
                        break
                
                await asyncio.sleep(frame_interval)
                
        except asyncio.CancelledError:
            logger.info(f"Streaming cancelled for {key}")
        except Exception as e:
            logger.error(f"Streaming error for {key}: {e}")
    
    async def handle_messages(self):
        """Handle incoming messages from central backend."""
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")
                    
                    if msg_type == "camera_command":
                        await self.handle_camera_command(data)
                    elif msg_type == "ping":
                        # Respond to ping
                        pong_msg = {
                            "type": "pong",
                            "timestamp": time.time()
                        }
                        await self.websocket.send(json.dumps(pong_msg))
                    else:
                        logger.info(f"Received unknown message type: {msg_type}")
                        
                except json.JSONDecodeError:
                    logger.warning("Received invalid JSON message")
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("Connection to central backend closed")
        except Exception as e:
            logger.error(f"Message handling error: {e}")
    
    async def run(self):
        """Run the mock Jetson server."""
        self.running = True
        
        # Connect to central backend
        if not await self.connect_to_central_backend():
            logger.error("Failed to connect to central backend")
            return
        
        try:
            # Send initial device discovery
            await self.send_device_discovery()
            
            # Start heartbeat task
            self.heartbeat_task = asyncio.create_task(self.send_heartbeat())
            
            # Handle incoming messages
            await self.handle_messages()
            
        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            await self.cleanup()
    
    async def cleanup(self):
        """Clean up resources."""
        logger.info("Cleaning up mock Jetson server...")
        self.running = False
        
        # Cancel all streaming tasks
        for task in self.streaming_tasks.values():
            task.cancel()
        
        # Cancel heartbeat task
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
        
        # Close WebSocket connection
        if self.websocket:
            await self.websocket.close()
        
        logger.info("Cleanup complete")

def signal_handler(signum, frame):
    """Handle interrupt signals."""
    logger.info("Received interrupt signal, shutting down...")
    sys.exit(0)

async def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Mock Jetson Server for Camera Testing")
    parser.add_argument(
        "--backend-url", 
        default="ws://localhost:8081/jetson",
        help="Central backend WebSocket URL"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run mock server
    server = MockJetsonServer(args.backend_url)
    
    logger.info("Starting Mock Jetson Server...")
    logger.info(f"Simulating {len(server.devices)} Jetson devices")
    
    for device_id, device in server.devices.items():
        logger.info(f"  {device_id}: {device.name} ({len(device.cameras)} cameras)")
    
    try:
        await server.run()
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
