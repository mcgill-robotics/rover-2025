#!/usr/bin/env python3
"""
Multi-camera UDP streaming server for Jetson/Pi devices.
Uses GStreamer for hardware-accelerated capture and H.264 encoding.
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
from typing import Dict, List
import logging
import signal

from config import get_jetson_config

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class CameraInfo:
    def __init__(self, camera_id: str, name: str, device_path: str):
        self.camera_id = camera_id
        self.name = name
        self.device_path = device_path
        self.is_active = False
        self.last_frame_time = 0
        self.gst_process = None
        self.rtp_port = None  # Port where RTP stream is sent to backend
        self.camera_type = None  # Type of camera/pipeline (MJPG, YUYV, RAW)

class MultiCameraStreamer:
    def __init__(self, backend_host: str, backend_port: int, device_id: str = "jetson-01"):
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.device_id = device_id
        self.cameras: Dict[str, CameraInfo] = {}
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)  # 1MB buffer
        
        # GStreamer settings
        self.width = 640
        self.height = 480
        self.framerate = 20
        self.bitrate = 512
        self.h264_tune = "zerolatency"
        
        # UDP frame capture settings
        self.frame_buffer_size = 65536
        self.max_frame_size = 60000
        
        # Command socket for receiving start/stop commands from backend
        self.command_sock = None
        self.command_port = backend_port + 1  # Use backend_port + 1 for commands
        
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
                            if video_num % 2 == 0:
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
        
    def build_gst_pipeline(self, camera_info: CameraInfo, udp_port: int) -> tuple[List[str], str]:
        """Build GStreamer pipeline and return (pipeline, camera_type)."""
        try:
            fmt_info = subprocess.check_output([
                "v4l2-ctl", "--device", camera_info.device_path, "--list-formats-ext"
            ], text=True)

            if 'MJPG' in fmt_info:
                logger.info(f"Using MJPG pipeline for {camera_info.device_path}")
                camera_info.camera_type = "MJPG"
                pipeline = [
                    "gst-launch-1.0",
                    "v4l2src", f"device={camera_info.device_path}",
                    "!", "image/jpeg,width=640,height=480,framerate=30/1",
                    "!", "jpegdec",
                    "!", "videoconvert",
                    "!", "video/x-raw,format=NV12",
                    "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
                    "!", "h264parse",
                    "!", "rtph264pay", "config-interval=1", "pt=96",
                    "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
                ]
                return pipeline, "MJPG"
            elif 'YUYV' in fmt_info or 'YUYV8' in fmt_info:
                logger.info(f"Using YUYV pipeline for {camera_info.device_path}")
                camera_info.camera_type = "YUYV"
                pipeline = [
                    "gst-launch-1.0",
                    "v4l2src", f"device={camera_info.device_path}",
                    "!", "video/x-raw,format=YUY2,width=640,height=480,framerate=20/1",
                    "!", "videoconvert",
                    "!", "video/x-raw,format=NV12",
                    "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
                    "!", "h264parse",
                    "!", "rtph264pay", "config-interval=1", "pt=96",
                    "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
                ]
                return pipeline, "YUYV"
            else:
                logger.warning(f"Unknown formats for {camera_info.device_path}, using fallback raw pipeline")

        except Exception as e:
            logger.warning(f"Could not query format for {camera_info.device_path}, using fallback pipeline: {e}")

        camera_info.camera_type = "RAW"
        pipeline = [
            "gst-launch-1.0",
            "v4l2src", f"device={camera_info.device_path}",
            "!", f"video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1",
            "!", "videoconvert",
            "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
            "!", "h264parse",
            "!", "rtph264pay", "config-interval=1", "pt=96",
            "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
        ]
        return pipeline, "RAW"
    
    def get_free_udp_port(self) -> int:
        """Get a free UDP port for GStreamer output."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(('', 0))
            return s.getsockname()[1]
    
    def send_frame_packet(self, camera_id: str, frame_data: bytes, packet_num: int, total_packets: int):
        """Send a single frame packet over UDP."""
        try:
            # Packet format: [camera_id_len][packet_num][total_packets][timestamp][camera_id][data]
            camera_id_bytes = camera_id.encode('utf-8')
            camera_id_len = len(camera_id_bytes)
            timestamp = int(time.time() * 1000)  # milliseconds
            
            header = struct.pack('!BHHQ', camera_id_len, packet_num, total_packets, timestamp)
            packet = header + camera_id_bytes + frame_data
            
            self.sock.sendto(packet, (self.backend_host, self.backend_port))
            
        except Exception as e:
            logger.error(f"Failed to send packet for {camera_id}: {e}")
    
    def send_frame(self, camera_id: str, frame_data: bytes):
        """Send a frame over UDP, splitting into packets if necessary."""
        frame_size = len(frame_data)
        
        if frame_size <= self.max_frame_size - 100:  # Account for header
            # Single packet
            self.send_frame_packet(camera_id, frame_data, 0, 1)
        else:
            # Multiple packets
            chunk_size = self.max_frame_size - 100
            total_packets = (frame_size + chunk_size - 1) // chunk_size
            
            for i in range(total_packets):
                start_idx = i * chunk_size
                end_idx = min(start_idx + chunk_size, frame_size)
                chunk = frame_data[start_idx:end_idx]
                self.send_frame_packet(camera_id, chunk, i, total_packets)
    
    def capture_h264_stream(self, camera_info: CameraInfo, udp_port: int):
        """Capture H.264 stream from GStreamer UDP output."""
        logger.info(f"Starting H.264 capture for {camera_info.camera_id} on port {udp_port}")
        
        # Create UDP socket to receive H.264 stream from GStreamer
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.frame_buffer_size)
        
        try:
            recv_sock.bind(('127.0.0.1', udp_port))
            recv_sock.settimeout(1.0)  # 1 second timeout
            
            frame_buffer = b''
            last_frame_time = time.time()
            
            while self.running and camera_info.is_active:
                try:
                    # Receive RTP packet from GStreamer
                    data, addr = recv_sock.recvfrom(self.frame_buffer_size)
                    
                    if len(data) < 12:  # Minimum RTP header size
                        continue
                    
                    # Parse RTP header
                    rtp_header = struct.unpack('!BBHII', data[:12])
                    version = (rtp_header[0] >> 6) & 0x3
                    marker = (rtp_header[1] >> 7) & 0x1
                    payload_type = rtp_header[1] & 0x7F
                    
                    if version != 2 or payload_type != 96:  # RTP version 2, PT 96 for H.264
                        continue
                    
                    # Extract H.264 payload (skip RTP header)
                    h264_payload = data[12:]
                    
                    # Accumulate H.264 data
                    frame_buffer += h264_payload
                    
                    # If marker bit is set, we have a complete frame
                    if marker:
                        current_time = time.time()
                        
                        # Send complete H.264 frame
                        if frame_buffer:
                            self.send_frame(camera_info.camera_id, frame_buffer)
                            camera_info.last_frame_time = current_time
                            
                            # Log frame rate occasionally
                            if current_time - last_frame_time > 5.0:
                                logger.debug(f"Camera {camera_info.camera_id} streaming H.264 frames")
                                last_frame_time = current_time
                        
                        # Reset buffer for next frame
                        frame_buffer = b''
                
                except socket.timeout:
                    # Check if we should continue
                    continue
                except Exception as e:
                    logger.error(f"Error receiving H.264 stream for {camera_info.camera_id}: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            logger.error(f"Failed to setup H.264 capture for {camera_info.camera_id}: {e}")
        finally:
            recv_sock.close()
            logger.info(f"Stopped H.264 capture for {camera_info.camera_id}")
    
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
            
            # Get free UDP port for local GStreamer capture
            local_udp_port = self.get_free_udp_port()
            
            # Build GStreamer pipeline that sends RTP to backend
            pipeline, camera_type = self.build_gst_pipeline(camera_info, rtp_port)
            
            logger.info(f"Starting GStreamer for {camera_id} -> RTP port {rtp_port}: {' '.join(pipeline)}")
            
            # Start GStreamer process
            camera_info.gst_process = subprocess.Popen(
                pipeline,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            # Wait a moment for GStreamer to start
            time.sleep(2)
            
            # Check if process is still running
            if camera_info.gst_process.poll() is not None:
                # Get error output from GStreamer
                stdout, stderr = camera_info.gst_process.communicate()
                logger.error(f"GStreamer process failed to start for {camera_id}")
                if stderr:
                    logger.error(f"GStreamer stderr: {stderr.decode('utf-8', errors='ignore')}")
                if stdout:
                    logger.error(f"GStreamer stdout: {stdout.decode('utf-8', errors='ignore')}")
                camera_info.rtp_port = None
                return False
            
            # Mark camera as active
            camera_info.is_active = True
            camera_info.last_frame_time = time.time()
            
            logger.info(f"Started streaming from {camera_id} to backend port {rtp_port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start camera {camera_id}: {e}")
            camera_info.rtp_port = None
            return False
    
    def stop_camera(self, camera_id: str):
        """Stop streaming from a specific camera."""
        if camera_id not in self.cameras:
            return
            
        camera_info = self.cameras[camera_id]
        camera_info.is_active = False
        
        # Stop GStreamer process
        if camera_info.gst_process:
            try:
                # Try graceful termination first
                camera_info.gst_process.terminate()
                camera_info.gst_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if graceful termination fails
                camera_info.gst_process.kill()
                camera_info.gst_process.wait()
            except Exception as e:
                logger.error(f"Error stopping GStreamer for {camera_id}: {e}")
            finally:
                camera_info.gst_process = None
        
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
            self.command_sock.settimeout(1.0)  # 1 second timeout
            logger.info(f"Command socket listening on port {self.command_port}")
        except Exception as e:
            logger.error(f"Failed to setup command socket: {e}")
            self.command_sock = None

    def handle_command(self, command_data: dict):
        """Handle command from backend."""
        try:
            command_type = command_data.get('type')
            camera_id = command_data.get('camera_id')
            
            if command_type == 'start_camera':
                logger.info(f"Received start command for camera {camera_id}")
                success = self.start_camera(camera_id)
                response = {
                    'type': 'command_response',
                    'command': 'start_camera',
                    'camera_id': camera_id,
                    'success': success,
                    'device_id': self.device_id
                }
                self.send_command_response(response)
                
            elif command_type == 'stop_camera':
                logger.info(f"Received stop command for camera {camera_id}")
                self.stop_camera(camera_id)
                response = {
                    'type': 'command_response',
                    'command': 'stop_camera',
                    'camera_id': camera_id,
                    'success': True,
                    'device_id': self.device_id
                }
                self.send_command_response(response)
                
            else:
                logger.warning(f"Unknown command type: {command_type}")
                
        except Exception as e:
            logger.error(f"Failed to handle command: {e}")

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
        max_failures = 3
        
        while self.running:
            try:
                camera_status = {}
                for camera_id, camera_info in self.cameras.items():
                    camera_status[camera_id] = {
                        'name': camera_info.name,
                        'device_path': camera_info.device_path,
                        'is_active': camera_info.is_active,
                        'last_frame_time': camera_info.last_frame_time,
                        'rtp_port': camera_info.rtp_port,  # Include RTP port for backend
                        'camera_type': camera_info.camera_type  # Include camera type (MJPG, YUYV, RAW)
                    }
                
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
                logger.debug(f"Sent heartbeat to {self.backend_host}:{self.backend_port} with {len(camera_status)} cameras")
                
            except Exception as e:
                heartbeat_failures += 1
                logger.error(f"Failed to send heartbeat ({heartbeat_failures}/{max_failures}): {e}")
                
                if heartbeat_failures >= max_failures:
                    logger.error(f"Too many heartbeat failures. Check network connectivity to {self.backend_host}:{self.backend_port}")
                    # Continue trying but with longer intervals
                    time.sleep(15)
                    heartbeat_failures = 0  # Reset counter
                
            time.sleep(5)  # Send heartbeat every 5 seconds
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False
    
    def run(self):
        """Main run loop."""
        logger.info(f"Starting multi-camera GStreamer streamer service (device: {self.device_id})")
        logger.info(f"Backend: {self.backend_host}:{self.backend_port}")
        logger.info(f"Command port: {self.command_port}")
        logger.info(f"Video settings: {self.width}x{self.height}@{self.framerate}fps, bitrate={self.bitrate}kbps")
        
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
        logger.info("Jetson camera service ready. Cameras will start when requested by backend.")
        
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
    parser.add_argument('--backend-host', default=config["DEFAULT_BACKEND_HOST"], 
                       help='Backend server IP address')
    parser.add_argument('--backend-port', type=int, default=config["DEFAULT_BACKEND_PORT"], 
                       help='Backend server UDP port')
    parser.add_argument('--device-id', default='jetson-01', 
                       help='Unique device identifier')
    parser.add_argument('--width', type=int, default=config["CAPTURE_WIDTH"], 
                       help='Video capture width')
    parser.add_argument('--height', type=int, default=config["CAPTURE_HEIGHT"], 
                       help='Video capture height')
    parser.add_argument('--fps', type=int, default=config["DEFAULT_FPS"], 
                       help='Target frames per second')
    parser.add_argument('--bitrate', type=int, default=512, 
                       help='H.264 bitrate in kbps')
    parser.add_argument('--tune', default='zerolatency', 
                       help='H.264 encoder tune setting')
    
    args = parser.parse_args()
    
    streamer = MultiCameraStreamer(
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        device_id=args.device_id
    )
    
    # Apply settings
    streamer.width = args.width
    streamer.height = args.height
    streamer.framerate = args.fps
    streamer.bitrate = args.bitrate
    streamer.h264_tune = args.tune
    
    streamer.run()

if __name__ == "__main__":
    import os
    main()
