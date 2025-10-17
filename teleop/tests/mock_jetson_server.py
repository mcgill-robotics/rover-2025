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
- Sends UDP heartbeats to central backend
- Responds to start/stop commands via UDP
- Simulates camera streaming when requested
"""

import asyncio
import json
import time
import threading
import cv2
import numpy as np
import base64
import logging
import socket
import struct
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
    rtp_port: Optional[int] = None
    device_path: str = ""

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
    
    def __init__(self, backend_host: str = "localhost", backend_port: int = 9999, num_devices: int = 3, cameras_per_device: int = 2):
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.num_devices = num_devices
        self.cameras_per_device = cameras_per_device
        self.devices = self._create_mock_devices()
        self.video_generators = {}
        self.streaming_tasks = {}
        self.running = False
        self.heartbeat_task = None
        self.udp_socket = None
        self.command_socket = None
        self.command_port = backend_port + 1
        
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
        
        patterns = ["color_bars", "checkerboard", "gradient", "noise"]
        resolutions = [(640, 480), (1280, 720), (320, 240)]
        device_names = ["Front Camera Unit", "Side Camera Unit", "Rear Camera Unit", "Aux Camera Unit", "Mobile Camera Unit"]
        
        for i in range(self.num_devices):
            device_id = f"jetson-{i+1:02d}"
            device_name = device_names[i % len(device_names)]
            ip_address = f"192.168.1.{101 + i}"
            
            cameras = []
            for j in range(self.cameras_per_device):
                camera_id = f"cam_{j}"
                camera_name = f"Camera {j+1}"
                resolution = resolutions[j % len(resolutions)]
                pattern = patterns[(i + j) % len(patterns)]
                fps = 30 if resolution[0] >= 640 else 15
                device_path = f"/dev/video{i*self.cameras_per_device + j}"
                
                cameras.append(CameraInfo(
                    camera_id=camera_id,
                    name=camera_name,
                    resolution=resolution,
                    fps=fps,
                    video_pattern=pattern,
                    device_path=device_path
                ))
            
            devices[device_id] = JetsonDevice(
                device_id=device_id,
                name=device_name,
                ip_address=ip_address,
                port=8080,
                cameras=cameras
            )
        
        return devices
    
    def create_udp_packet(self, camera_id: str, payload: bytes) -> bytes:
        """Create UDP packet in the format expected by central backend."""
        camera_id_bytes = camera_id.encode('utf-8')
        camera_id_len = len(camera_id_bytes)
        
        # Header: [camera_id_len][packet_num][total_packets][timestamp]
        packet_num = 1
        total_packets = 1
        timestamp = int(time.time() * 1000)
        
        header = struct.pack('!BHHQ', camera_id_len, packet_num, total_packets, timestamp)
        
        return header + camera_id_bytes + payload
    
    def send_heartbeat(self):
        """Send heartbeat for all devices."""
        if not self.udp_socket:
            return
            
        for device in self.devices.values():
            if device.is_online:
                # Create heartbeat data
                cameras_data = {}
                for camera in device.cameras:
                    cameras_data[f"{device.device_id}-{camera.camera_id}"] = {
                        "name": camera.name,
                        "device_path": camera.device_path,
                        "is_active": camera.is_active,
                        "rtp_port": camera.rtp_port if camera.is_active else None
                    }
                
                heartbeat_data = {
                    "device_id": device.device_id,
                    "name": device.name,
                    "ip_address": device.ip_address,
                    "cameras": cameras_data,
                    "timestamp": time.time()
                }
                
                # Send as UDP packet
                try:
                    payload = json.dumps(heartbeat_data).encode('utf-8')
                    packet = self.create_udp_packet("__HEARTBEAT__", payload)
                    
                    self.udp_socket.sendto(packet, (self.backend_host, self.backend_port))
                    device.last_heartbeat = time.time()
                    logger.debug(f"Sent heartbeat for device {device.device_id}")
                    
                except Exception as e:
                    logger.error(f"Failed to send heartbeat for {device.device_id}: {e}")
    
    def setup_command_listener(self):
        """Set up UDP socket to listen for commands from central backend."""
        try:
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_socket.bind(('0.0.0.0', self.command_port))
            self.command_socket.settimeout(1.0)
            logger.info(f"Command listener started on port {self.command_port}")
        except Exception as e:
            logger.error(f"Failed to setup command listener: {e}")
            self.command_socket = None
    
    def handle_command(self, command_data: dict):
        """Handle command from central backend."""
        try:
            command_type = command_data.get("type")
            camera_id = command_data.get("camera_id")
            
            if command_type == "start_camera":
                self.start_camera(camera_id)
            elif command_type == "stop_camera":
                self.stop_camera(camera_id)
            else:
                logger.warning(f"Unknown command type: {command_type}")
                
        except Exception as e:
            logger.error(f"Failed to handle command: {e}")
    
    def start_camera(self, full_camera_id: str):
        """Start a camera (e.g., 'jetson-01-cam_0')."""
        try:
            # Parse camera ID to get device and camera
            parts = full_camera_id.split('-')
            if len(parts) < 3:
                logger.error(f"Invalid camera ID format: {full_camera_id}")
                return
                
            device_id = '-'.join(parts[:-1])  # e.g., "jetson-01"
            camera_id = parts[-1]  # e.g., "cam_0"
            
            if device_id not in self.devices:
                logger.error(f"Device not found: {device_id}")
                return
                
            device = self.devices[device_id]
            camera = None
            
            for cam in device.cameras:
                if cam.camera_id == camera_id:
                    camera = cam
                    break
            
            if not camera:
                logger.error(f"Camera not found: {camera_id} on device {device_id}")
                return
            
            if not camera.is_active:
                # Assign RTP port (simulate starting GStreamer pipeline)
                camera.rtp_port = 5000 + len([c for c in device.cameras if c.is_active])
                camera.is_active = True
                
                logger.info(f"Started camera {full_camera_id} on RTP port {camera.rtp_port}")
                
                # Start streaming simulation (optional - for testing)
                key = f"{device_id}_{camera_id}"
                if key not in self.streaming_tasks:
                    task = threading.Thread(
                        target=self.simulate_camera_stream,
                        args=(device_id, camera_id, camera),
                        daemon=True
                    )
                    task.start()
                    self.streaming_tasks[key] = task
            
        except Exception as e:
            logger.error(f"Failed to start camera {full_camera_id}: {e}")
    
    def stop_camera(self, full_camera_id: str):
        """Stop a camera."""
        try:
            # Parse camera ID
            parts = full_camera_id.split('-')
            if len(parts) < 3:
                logger.error(f"Invalid camera ID format: {full_camera_id}")
                return
                
            device_id = '-'.join(parts[:-1])
            camera_id = parts[-1]
            
            if device_id not in self.devices:
                logger.error(f"Device not found: {device_id}")
                return
                
            device = self.devices[device_id]
            camera = None
            
            for cam in device.cameras:
                if cam.camera_id == camera_id:
                    camera = cam
                    break
            
            if not camera:
                logger.error(f"Camera not found: {camera_id} on device {device_id}")
                return
            
            if camera.is_active:
                camera.is_active = False
                camera.rtp_port = None
                
                logger.info(f"Stopped camera {full_camera_id}")
                
                # Stop streaming simulation
                key = f"{device_id}_{camera_id}"
                if key in self.streaming_tasks:
                    # Note: Thread will stop when camera.is_active becomes False
                    del self.streaming_tasks[key]
            
        except Exception as e:
            logger.error(f"Failed to stop camera {full_camera_id}: {e}")
    
    def simulate_camera_stream(self, device_id: str, camera_id: str, camera: CameraInfo):
        """Simulate camera streaming (for testing purposes)."""
        key = f"{device_id}_{camera_id}"
        generator = self.video_generators[key]
        
        frame_interval = 1.0 / camera.fps
        
        logger.info(f"Started streaming simulation for {device_id}-{camera_id}")
        
        try:
            while camera.is_active and self.running:
                # Generate frame (this simulates the camera producing frames)
                frame = generator.get_frame(camera.video_pattern)
                
                # In a real implementation, this would be sent via GStreamer RTP
                # For now, we just simulate the frame generation
                
                time.sleep(frame_interval)
                
        except Exception as e:
            logger.error(f"Streaming simulation error for {key}: {e}")
        
        logger.info(f"Stopped streaming simulation for {device_id}-{camera_id}")
    
    def command_listener_thread(self):
        """Thread to listen for commands from central backend."""
        logger.info("Starting command listener thread")
        
        while self.running:
            if not self.command_socket:
                time.sleep(1)
                continue
                
            try:
                data, addr = self.command_socket.recvfrom(1024)
                logger.debug(f"Received command from {addr}: {data}")
                
                try:
                    command_data = json.loads(data.decode('utf-8'))
                    self.handle_command(command_data)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON command from {addr}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Command listener error: {e}")
                time.sleep(0.1)
        
        logger.info("Command listener thread ended")
    
    def heartbeat_thread(self):
        """Thread to send periodic heartbeats."""
        logger.info("Starting heartbeat thread")
        
        while self.running:
            try:
                self.send_heartbeat()
                time.sleep(5)  # Send heartbeat every 5 seconds
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                time.sleep(5)
        
        logger.info("Heartbeat thread ended")
    
    def run(self):
        """Run the mock Jetson server."""
        self.running = True
        
        try:
            # Setup UDP socket for sending heartbeats
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # Setup command listener
            self.setup_command_listener()
            
            # Start command listener thread
            if self.command_socket:
                command_thread = threading.Thread(target=self.command_listener_thread, daemon=True)
                command_thread.start()
            
            # Start heartbeat thread
            heartbeat_thread = threading.Thread(target=self.heartbeat_thread, daemon=True)
            heartbeat_thread.start()
            
            logger.info("Mock Jetson server started")
            logger.info(f"Sending heartbeats to {self.backend_host}:{self.backend_port}")
            logger.info(f"Listening for commands on port {self.command_port}")
            
            # Keep main thread alive
            while self.running:
                time.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Received interrupt signal")
        except Exception as e:
            logger.error(f"Server error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        logger.info("Cleaning up mock Jetson server...")
        self.running = False
        
        # Close sockets
        if self.udp_socket:
            self.udp_socket.close()
        if self.command_socket:
            self.command_socket.close()
        
        logger.info("Cleanup complete")

def signal_handler(signum, frame):
    """Handle interrupt signals."""
    logger.info("Received interrupt signal, shutting down...")
    sys.exit(0)

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Mock Jetson Server for Camera Testing")
    parser.add_argument(
        "--backend-host", 
        default="localhost",
        help="Central backend host"
    )
    parser.add_argument(
        "--backend-port", 
        type=int,
        default=9999,
        help="Central backend UDP port"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    parser.add_argument(
        "--num-devices",
        type=int,
        default=3,
        help="Number of Jetson devices to simulate"
    )
    parser.add_argument(
        "--cameras-per-device",
        type=int,
        default=2,
        help="Number of cameras per device"
    )
    
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run mock server
    server = MockJetsonServer(args.backend_host, args.backend_port, args.num_devices, args.cameras_per_device)
    
    logger.info("Starting Mock Jetson Server...")
    logger.info(f"Simulating {len(server.devices)} Jetson devices")
    
    for device_id, device in server.devices.items():
        logger.info(f"  {device_id}: {device.name} ({len(device.cameras)} cameras)")
    
    try:
        server.run()
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server error: {e}")

if __name__ == "__main__":
    main()
