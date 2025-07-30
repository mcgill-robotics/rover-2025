#!/usr/bin/env python3
"""
Multi-camera UDP streaming server for Jetson/Pi devices.
Uses GStreamer for hardware-accelerated capture and H.264 encoding.
Sends encoded frames with camera IDs over UDP to central backend.
"""

import asyncio
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

class MultiCameraStreamer:
    def __init__(self, backend_host: str, backend_port: int, device_id: str = "jetson-01"):
        self.backend_host = backend_host
        self.backend_port = backend_port
        self.device_id = device_id
        self.cameras: Dict[str, CameraInfo] = {}
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)

        # GStreamer settings
        self.width = 640
        self.height = 480
        self.framerate = 20
        self.bitrate = 512
        self.h264_tune = "zerolatency"

        self.frame_buffer_size = 65536
        self.max_frame_size = 60000

    def signal_handler(self, signum, frame):
        logger.info(f"Received signal {signum}, shutting down...")
        self.running = False

    def send_heartbeat(self):
        while self.running:
            try:
                message = json.dumps({
                    "device_id": self.device_id,
                    "timestamp": time.time(),
                    "event": "heartbeat"
                }).encode("utf-8")
                self.sock.sendto(message, (self.backend_host, self.backend_port))
            except Exception as e:
                logger.warning(f"Failed to send heartbeat: {e}")
            time.sleep(5)

    def discover_cameras(self) -> List[CameraInfo]:
        cameras = []
        try:
            result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=10)
            output = result.stdout.strip().splitlines()
            current_name = None
            device_counter = 0

            for line in output:
                if not line.startswith("\t") and line.strip():
                    current_name = re.sub(r"\s\(.+\):?$", "", line.strip())
                elif line.startswith("\t") and current_name:
                    device_path = line.strip()
                    if device_path.startswith("/dev/video"):
                        match = re.search(r'/dev/video(\d+)', device_path)
                        if match:
                            num = int(match.group(1))
                            if num % 2 == 0:
                                camera_id = f"{self.device_id}-cam{device_counter:02d}"
                                camera_info = CameraInfo(camera_id, current_name, device_path)
                                cameras.append(camera_info)
                                device_counter += 1
                                logger.info(f"Found camera: {camera_id} - {current_name} ({device_path})")
        except Exception as e:
            logger.error(f"Failed to discover cameras: {e}")
        return cameras

    def build_gst_pipeline(self, camera_info: CameraInfo, udp_port: int) -> List[str]:
        try:
            fmt_info = subprocess.check_output([
                "v4l2-ctl", "--device", camera_info.device_path, "--list-formats-ext"
            ], text=True)

            if 'MJPG' in fmt_info:
                logger.info(f"Using MJPG pipeline for {camera_info.device_path}")
                return [
                    "gst-launch-1.0",
                    "v4l2src", f"device={camera_info.device_path}",
                    "!", "image/jpeg,width=640,height=480,framerate=30/1",
                    "!", "jpegdec",
                    "!", "nvvidconv",
                    "!", "video/x-raw,format=NV12",
                    "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
                    "!", "h264parse",
                    "!", "rtph264pay", "config-interval=1", "pt=96",
                    "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
                ]
            elif 'YUYV' in fmt_info or 'YUYV8' in fmt_info:
                logger.info(f"Using YUYV pipeline for {camera_info.device_path}")
                return [
                    "gst-launch-1.0",
                    "v4l2src", f"device={camera_info.device_path}",
                    "!", "video/x-raw,format=YUY2,width=640,height=480,framerate=20/1",
                    "!", "nvvidconv",
                    "!", "video/x-raw,format=NV12",
                    "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
                    "!", "h264parse",
                    "!", "rtph264pay", "config-interval=1", "pt=96",
                    "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
                ]
            else:
                logger.warning(f"Unknown formats for {camera_info.device_path}, using fallback raw pipeline")

        except Exception as e:
            logger.warning(f"Could not query format for {camera_info.device_path}, using fallback pipeline: {e}")

        return [
            "gst-launch-1.0",
            "v4l2src", f"device={camera_info.device_path}",
            "!", f"video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1",
            "!", "nvvidconv",
            "!", "x264enc", f"tune={self.h264_tune}", f"bitrate={self.bitrate}",
            "!", "h264parse",
            "!", "rtph264pay", "config-interval=1", "pt=96",
            "!", "udpsink", f"host={self.backend_host}", f"port={udp_port}"
        ]

    def run(self):
        logger.info(f"Starting multi-camera GStreamer streamer (device: {self.device_id})")
        logger.info(f"Backend: {self.backend_host}:{self.backend_port}")
        logger.info(f"Video settings: {self.width}x{self.height}@{self.framerate}fps, bitrate={self.bitrate}kbps")

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        discovered_cameras = self.discover_cameras()
        if not discovered_cameras:
            logger.error("No cameras found!")
            return

        for camera_info in discovered_cameras:
            self.cameras[camera_info.camera_id] = camera_info

        self.running = True

        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        self.start_all_cameras()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received...")
        finally:
            logger.info("Shutting down...")
            self.running = False
            self.stop_all_cameras()
            self.sock.close()

def main():
    config = get_jetson_config()

    parser = argparse.ArgumentParser(description='Multi-camera GStreamer UDP streamer for Jetson/Pi')
    parser.add_argument('--backend-host', default=config["DEFAULT_BACKEND_HOST"], help='Backend server IP address')
    parser.add_argument('--backend-port', type=int, default=config["DEFAULT_BACKEND_PORT"], help='Backend server UDP port')
    parser.add_argument('--device-id', default='jetson-01', help='Unique device identifier')
    parser.add_argument('--width', type=int, default=config["CAPTURE_WIDTH"], help='Video capture width')
    parser.add_argument('--height', type=int, default=config["CAPTURE_HEIGHT"], help='Video capture height')
    parser.add_argument('--fps', type=int, default=config["DEFAULT_FPS"], help='Target frames per second')
    parser.add_argument('--bitrate', type=int, default=512, help='H.264 bitrate in kbps')
    parser.add_argument('--tune', default='zerolatency', help='H.264 encoder tune setting')

    args = parser.parse_args()

    streamer = MultiCameraStreamer(
        backend_host=args.backend_host,
        backend_port=args.backend_port,
        device_id=args.device_id
    )

    streamer.width = args.width
    streamer.height = args.height
    streamer.framerate = args.fps
    streamer.bitrate = args.bitrate
    streamer.h264_tune = args.tune

    streamer.run()

if __name__ == "__main__":
    main()