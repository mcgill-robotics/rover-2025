#!/usr/bin/env python3
"""
Multi-camera UDP streaming server for Jetson/Pi devices.
Uses GStreamer Python bindings (gi) for hardware-accelerated capture and H.264 encoding.
Sends encoded frames with camera IDs over UDP to central backend.
"""

import json
import socket
import struct
import time
import threading
import subprocess
import re
import argparse
from typing import Dict, List, Optional
import logging
import signal
import os

from config import get_jetson_config, get_network_config, setup_logging
from gstreamer_pipeline import GStreamerPipeline

# Configure logging
setup_logging()
logger = logging.getLogger(__name__)

class CameraInfo:
    def __init__(self, camera_id: str, name: str, device_path: str):
        self.camera_id = camera_id
        self.name = name
        self.device_path = device_path
        self.is_active = False
        self.last_frame_time = 0
        self.pipeline: Optional[GStreamerPipeline] = None
        self.rtp_port = None  # Port where RTP stream is sent to backend
        self.camera_type = None  # Type of camera/pipeline (MJPG, YUYV, RAW)
        
        # Performance monitoring
        self.frame_count = 0
        self.start_time = 0
        self.current_fps = 0.0
        self.avg_latency = 0.0
        self.last_performance_update = 0

class MultiCameraStreamer:
    def __init__(self, backend_host: str, backend_port: int, device_id: str = "jetson-01"):
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.device_id = device_id
        self.cameras: Dict[str, CameraInfo] = {}
        self.running = False
        
        # Load configuration
        self.config = get_jetson_config()
        self.network_config = get_network_config()
        
        # Setup UDP socket for heartbeats
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_SNDBUF,
            self.network_config.get('udp_send_buffer_size', 1024 * 1024)
        )
        
        # Command socket for receiving start/stop commands from backend
        self.command_sock = None
        self.command_port = backend_port + self.network_config.get('command_port_offset', 1)
        
    def discover_cameras(self) -> List[CameraInfo]:
        """Discover available cameras using v4l2-ctl, only using even-numbered video devices."""
        cameras = []
        try:
            result = subprocess.run(["v4l2-ctl", "--list-devices"], 
                                  capture_output=True, text=True, timeout=10)
            output = result.stdout.strip().splitlines()
            
            current_name = None
            device_counter = 0
            
            for line in output:
                if not line.startswith("\t") and line.strip():
                    # Camera name line
                    current_name = re.sub(r"\s\(.+\):?$", "", line.strip())
                elif line.startswith("\t") and current_name:
                    # Device path line
                    device_path = line.strip()
                    if device_path.startswith("/dev/video"):
                        # Extract video device number
                        video_match = re.search(r'/dev/video(\d+)', device_path)
                        if video_match:
                            video_num = int(video_match.group(1))
                            # Only use even-numbered video devices (0, 2, 4, etc.)
                            # Odd numbers are typically metadata streams
                            if video_num % 2 == 0 and video_num < 10:
                                camera_id = f"{self.device_id}-cam{device_counter:02d}"
                                camera_info = CameraInfo(camera_id, current_name, device_path)
                                cameras.append(camera_info)
                                device_counter += 1
                                logger.info(f"Found camera: {camera_id} - {current_name} ({device_path})")
                            else:
                                logger.debug(f"Skipping odd video device (metadata stream): {device_path}")
                        
        except Exception as e:
            logger.error(f"Failed to discover cameras: {e}")
            
        return cameras
    
    def test_camera(self, device_path: str) -> bool:
        """Test if a camera device is accessible using v4l2-ctl."""
        try:
            result = subprocess.run(
                ["v4l2-ctl", "--device", device_path, "--info"],
                capture_output=True, text=True, timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"Failed to test camera {device_path}: {e}")
            return False
    
    def get_free_udp_port(self) -> int:
        """Get a free UDP port for GStreamer output."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('', 0))
            return s.getsockname()[1]
    
    def start_camera(self, camera_id: str) -> bool:
        """Start streaming from a specific camera using GStreamer."""
        if camera_id not in self.cameras:
            logger.error(f"Camera {camera_id} not found")
            return False
            
        camera_info = self.cameras[camera_id]
        
        if camera_info.is_active:
            logger.warning(f"Camera {camera_id} is already active")
            return True
            
        if not self.test_camera(camera_info.device_path):
            logger.error(f"Camera {camera_id} is not accessible")
            return False
        
        try:
            # Get free UDP port for RTP stream to backend
            rtp_port = self.get_free_udp_port()
            camera_info.rtp_port = rtp_port
            
            logger.info(f"[{camera_id}] [STARTUP] Starting camera with device {camera_info.device_path}")
            logger.info(f"[{camera_id}] [STARTUP] Assigned RTP port: {rtp_port}")
            logger.info(f"[{camera_id}] [STARTUP] Backend target: {self.backend_host}:{rtp_port}")
            
            # Create GStreamer pipeline
            pipeline = GStreamerPipeline(
                device_path=camera_info.device_path,
                camera_id=camera_id,
                host=self.backend_host,
                port=rtp_port
            )
            
            # Start pipeline
            if not pipeline.start():
                logger.error(f"[{camera_id}] [STARTUP] Failed to start GStreamer pipeline")
                camera_info.rtp_port = None
                return False
            
            # Store pipeline and update state
            camera_info.pipeline = pipeline
            camera_info.is_active = True
            camera_info.last_frame_time = time.time()
            camera_info.camera_type = pipeline.camera_type
            
            logger.info(f"[{camera_id}] [STARTUP] Successfully started streaming to backend port {rtp_port}")
            logger.info(f"[{camera_id}] [STARTUP] Camera type: {camera_info.camera_type}")
            return True
            
        except Exception as e:
            logger.error(f"[{camera_id}] [STARTUP] Failed to start camera: {e}")
            import traceback
            logger.error(f"[{camera_id}] [STARTUP] Stack trace: {traceback.format_exc()}")
            camera_info.rtp_port = None
            return False
    
    def stop_camera(self, camera_id: str):
        """Stop streaming from a specific camera."""
        if camera_id not in self.cameras:
            return
            
        camera_info = self.cameras[camera_id]
        camera_info.is_active = False
        
        # Stop GStreamer pipeline
        if camera_info.pipeline:
            try:
                camera_info.pipeline.stop()
            except Exception as e:
                logger.error(f"Error stopping GStreamer for {camera_id}: {e}")
            finally:
                camera_info.pipeline = None
        
        logger.info(f"Stopped streaming from {camera_id}")
    
    def start_all_cameras(self):
        """Start streaming from all discovered cameras."""
        for camera_id in self.cameras:
            self.start_camera(camera_id)
    
    def stop_all_cameras(self):
        """Stop streaming from all cameras."""
        for camera_id in list(self.cameras.keys()):
            self.stop_camera(camera_id)
    
    def setup_command_socket(self):
        """Setup UDP socket for receiving commands from backend."""
        try:
            self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_sock.bind(('0.0.0.0', self.command_port))
            self.command_sock.settimeout(self.network_config.get('command_timeout', 1.0))
            logger.info(f"Command socket listening on port {self.command_port}")
        except Exception as e:
            logger.error(f"Failed to setup command socket: {e}")
            self.command_sock = None
    
    def handle_command(self, command_data: dict):
        """Handle command from backend."""
        try:
            command_type = command_data.get('type')
            camera_id = command_data.get('camera_id')
            
            logger.info(f"[COMMAND] Received {command_type} command for camera {camera_id}")
            
            if command_type == 'start_camera':
                logger.info(f"[COMMAND] Starting camera {camera_id}...")
                success = self.start_camera(camera_id)
                if success:
                    logger.info(f"[COMMAND] Successfully started camera {camera_id}")
                else:
                    logger.error(f"[COMMAND] Failed to start camera {camera_id}")
                
                response = {
                    'type': 'command_response',
                    'command': 'start_camera',
                    'camera_id': camera_id,
                    'success': success,
                    'device_id': self.device_id
                }
                self.send_command_response(response)
                
            elif command_type == 'stop_camera':
                logger.info(f"[COMMAND] Stopping camera {camera_id}...")
                self.stop_camera(camera_id)
                logger.info(f"[COMMAND] Successfully stopped camera {camera_id}")
                
                response = {
                    'type': 'command_response',
                    'command': 'stop_camera',
                    'camera_id': camera_id,
                    'success': True,
                    'device_id': self.device_id
                }
                self.send_command_response(response)
                
            elif command_type == 'update_settings':
                logger.info(f"[COMMAND] Updating settings for camera {camera_id}")
                settings = command_data.get('settings', {})
                success = self.update_camera_settings(camera_id, settings)
                response = {
                    'type': 'command_response',
                    'command': 'update_settings',
                    'camera_id': camera_id,
                    'success': success,
                    'device_id': self.device_id,
                    'settings': settings
                }
                self.send_command_response(response)
                
            elif command_type == 'dynamic_update':
                logger.info(f"[COMMAND] Dynamic update for camera {camera_id}")
                property_name = command_data.get('property')
                value = command_data.get('value')
                success = self.update_camera_property_dynamic(camera_id, property_name, value)
                response = {
                    'type': 'command_response',
                    'command': 'dynamic_update',
                    'camera_id': camera_id,
                    'success': success,
                    'device_id': self.device_id,
                    'property': property_name,
                    'value': value
                }
                self.send_command_response(response)
                
            else:
                logger.warning(f"[COMMAND] Unknown command type: {command_type}")
                
        except Exception as e:
            logger.error(f"[COMMAND] Failed to handle command: {e}")
            import traceback
            logger.error(f"[COMMAND] Stack trace: {traceback.format_exc()}")
    
    def update_camera_settings(self, camera_id: str, settings: dict) -> bool:
        """Update camera settings (bitrate, fps, etc.)."""
        try:
            if camera_id not in self.cameras:
                logger.error(f"Camera {camera_id} not found for settings update")
                return False
            
            camera_info = self.cameras[camera_id]
            if not camera_info.pipeline:
                logger.error(f"No active pipeline for camera {camera_id}")
                return False
            
            success = True
            
            # Update bitrate if provided
            if 'bitrate' in settings:
                success = success and camera_info.pipeline.update_bitrate(settings['bitrate'])
            
            # Update FPS if provided
            if 'fps' in settings:
                success = success and camera_info.pipeline.update_framerate(settings['fps'])
            
            return success
            
        except Exception as e:
            logger.error(f"Failed to update camera settings: {e}")
            return False
    
    def update_camera_property_dynamic(self, camera_id: str, property_name: str, value) -> bool:
        """Dynamically update camera property without restarting pipeline."""
        try:
            if camera_id not in self.cameras:
                logger.error(f"Camera {camera_id} not found for dynamic update")
                return False
            
            camera_info = self.cameras[camera_id]
            if not camera_info.pipeline:
                logger.error(f"No active pipeline for camera {camera_id}")
                return False
            
            if property_name == 'bitrate':
                return camera_info.pipeline.update_bitrate(value)
            elif property_name == 'fps':
                return camera_info.pipeline.update_framerate(value)
            else:
                logger.warning(f"Unknown property for dynamic update: {property_name}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to update camera property dynamically: {e}")
            return False
    
    def update_camera_performance(self, camera_id: str):
        """Update performance statistics for a camera."""
        if camera_id not in self.cameras:
            return
            
        camera_info = self.cameras[camera_id]
        if not camera_info.is_active:
            return
            
        current_time = time.time()
        
        # Update frame count and calculate FPS
        if camera_info.start_time == 0:
            camera_info.start_time = current_time
        
        elapsed_time = current_time - camera_info.start_time
        if elapsed_time > 0:
            camera_info.current_fps = camera_info.frame_count / elapsed_time
        
        # Log performance every 10 seconds
        if current_time - camera_info.last_performance_update > 10:
            logger.info(f"[{camera_id}] [PERFORMANCE] FPS: {camera_info.current_fps:.1f}, "
                       f"Frames: {camera_info.frame_count}, "
                       f"Runtime: {elapsed_time:.1f}s")
            camera_info.last_performance_update = current_time
    
    def get_system_health(self) -> dict:
        """Get system health status."""
        active_cameras = sum(1 for cam in self.cameras.values() if cam.is_active)
        total_cameras = len(self.cameras)
        
        health_status = {
            'device_id': self.device_id,
            'status': 'healthy' if self.running else 'stopped',
            'uptime': time.time() - getattr(self, '_start_time', time.time()),
            'cameras': {
                'total': total_cameras,
                'active': active_cameras,
                'inactive': total_cameras - active_cameras
            },
            'network': {
                'backend_host': self.backend_host,
                'backend_port': self.backend_port,
                'command_port': self.command_port
            },
            'performance': {}
        }
        
        # Add camera-specific performance data
        for camera_id, camera_info in self.cameras.items():
            if camera_info.is_active:
                health_status['performance'][camera_id] = {
                    'fps': camera_info.current_fps,
                    'frame_count': camera_info.frame_count,
                    'rtp_port': camera_info.rtp_port,
                    'camera_type': camera_info.camera_type
                }
        
        return health_status
    
    def send_command_response(self, response_data: dict):
        """Send command response back to backend."""
        try:
            response_json = json.dumps(response_data).encode('utf-8')
            
            # Send response with special camera_id
            camera_id_bytes = b'__COMMAND_RESPONSE__'
            camera_id_len = len(camera_id_bytes)
            timestamp = int(time.time() * 1000)
            
            header = struct.pack('!BHHQ', camera_id_len, 0, 1, timestamp)
            packet = header + camera_id_bytes + response_json
            
            self.sock.sendto(packet, (self.backend_host, self.backend_port))
            logger.debug(f"Sent command response: {response_data}")
            
        except Exception as e:
            logger.error(f"Failed to send command response: {e}")
    
    def command_listener_thread(self):
        """Thread to listen for commands from backend."""
        logger.info("Starting command listener thread")
        
        while self.running:
            if not self.command_sock:
                time.sleep(1)
                continue
                
            try:
                data, addr = self.command_sock.recvfrom(65536)
                logger.debug(f"Received command from {addr}, size: {len(data)} bytes")
                
                try:
                    command_data = json.loads(data.decode('utf-8'))
                    self.handle_command(command_data)
                except json.JSONDecodeError as e:
                    logger.error(f"Failed to parse command JSON: {e}")
                    
            except socket.timeout:
                # Normal timeout, continue
                continue
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    logger.debug(f"Command receive error: {e}")
                time.sleep(0.1)
        
        logger.info("Command listener thread ended")
    
    def send_heartbeat(self):
        """Send periodic heartbeat with camera status including RTP port information."""
        heartbeat_failures = 0
        max_failures = self.network_config.get('max_retries', 3)
        heartbeat_interval = self.config.get('heartbeat_interval', 5.0)
        
        while self.running:
            try:
                camera_status = {}
                active_cameras = 0
                rtp_ports = []
                
                for camera_id, camera_info in self.cameras.items():
                    camera_status[camera_id] = {
                        'name': camera_info.name,
                        'device_path': camera_info.device_path,
                        'is_active': camera_info.is_active,
                        'last_frame_time': camera_info.last_frame_time,
                        'rtp_port': camera_info.rtp_port,
                        'camera_type': camera_info.camera_type
                    }
                    
                    if camera_info.is_active:
                        active_cameras += 1
                        if camera_info.rtp_port:
                            rtp_ports.append(f"{camera_id}:{camera_info.rtp_port}")
                
                heartbeat_data = {
                    'type': 'heartbeat',
                    'device_id': self.device_id,
                    'timestamp': time.time(),
                    'cameras': camera_status
                }
                
                heartbeat_json = json.dumps(heartbeat_data).encode('utf-8')
                
                # Send heartbeat with special camera_id
                camera_id_bytes = b'__HEARTBEAT__'
                camera_id_len = len(camera_id_bytes)
                timestamp = int(time.time() * 1000)
                
                header = struct.pack('!BHHQ', camera_id_len, 0, 1, timestamp)
                packet = header + camera_id_bytes + heartbeat_json
                
                self.sock.sendto(packet, (self.backend_host, self.backend_port))
                
                # Reset failure counter on successful send
                heartbeat_failures = 0
                
                # Enhanced logging
                if active_cameras > 0:
                    logger.info(f"[HEARTBEAT] Sent to {self.backend_host}:{self.backend_port} - {active_cameras} active cameras")
                    logger.info(f"[HEARTBEAT] RTP ports: {', '.join(rtp_ports)}")
                else:
                    logger.debug(f"[HEARTBEAT] Sent to {self.backend_host}:{self.backend_port} - {len(camera_status)} cameras (none active)")
                
            except Exception as e:
                heartbeat_failures += 1
                logger.error(f"[HEARTBEAT] Failed to send heartbeat ({heartbeat_failures}/{max_failures}): {e}")
                
                if heartbeat_failures >= max_failures:
                    logger.error(f"[HEARTBEAT] Too many heartbeat failures. Check network connectivity to {self.backend_host}:{self.backend_port}")
                    # Continue trying but with longer intervals
                    time.sleep(15)
                    heartbeat_failures = 0  # Reset counter
                
            time.sleep(heartbeat_interval)
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False
    
    def run(self):
        """Main run loop."""
        logger.info(f"Starting multi-camera GStreamer streamer service (device: {self.device_id})")
        logger.info(f"Backend: {self.backend_host}:{self.backend_port}")
        logger.info(f"Command port: {self.command_port}")
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Discover cameras
        discovered_cameras = self.discover_cameras()
        if not discovered_cameras:
            logger.error("No cameras found!")
            return
            
        # Add cameras to our registry (but don't start them yet)
        for camera_info in discovered_cameras:
            self.cameras[camera_info.camera_id] = camera_info
        
        logger.info(f"Discovered {len(self.cameras)} cameras. Waiting for start commands from backend.")
        
        self.running = True
        self._start_time = time.time()  # Track uptime for health monitoring
        
        # Setup command socket
        self.setup_command_socket()
        
        # Start heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()
        
        # Start command listener thread
        command_thread = threading.Thread(target=self.command_listener_thread)
        command_thread.daemon = True
        command_thread.start()
        
        # Don't start cameras automatically - wait for commands from backend
        logger.info("Vision camera service ready. Cameras will start when requested by backend.")
        
        try:
            # Keep main thread alive
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received...")
        finally:
            logger.info("Shutting down...")
            self.running = False
            self.stop_all_cameras()
            if self.command_sock:
                self.command_sock.close()
            self.sock.close()

def main():
    # Load config defaults
    config = get_jetson_config()
    
    parser = argparse.ArgumentParser(description='Multi-camera GStreamer UDP streamer for Jetson/Pi')
    parser.add_argument('--backend-host', default=config["default_backend_host"], 
                       help='Backend server IP address')
    parser.add_argument('--backend-port', type=int, default=config["default_backend_port"], 
                       help='Backend server UDP port')
    parser.add_argument('--device-id', default='jetson-01', 
                       help='Unique device identifier')
    
    args = parser.parse_args()
    
    streamer = MultiCameraStreamer(
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        device_id=args.device_id
    )
    
    streamer.run()

if __name__ == "__main__":
    main()