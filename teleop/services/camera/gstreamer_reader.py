#!/usr/bin/env python3
"""
GStreamer-based camera reader for receiving RTP/H.264 streams via UDP.
Uses GStreamer Python bindings (gi) instead of cv2.VideoCapture for reliable UDP stream handling.
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import numpy as np
import logging
import time
import threading
from typing import Optional, Dict
from queue import Queue, Empty

from config import get_backend_config

# Initialize GStreamer
Gst.init(None)

# Configure logging with colors
class ColoredFormatter(logging.Formatter):
    """Custom formatter with colors for different log levels and components."""
    
    # Colors
    COLORS = {
        'DEBUG': '\033[36m',    # Cyan
        'INFO': '\033[32m',     # Green
        'WARNING': '\033[33m',  # Yellow
        'ERROR': '\033[31m',    # Red
        'CRITICAL': '\033[35m', # Magenta
        'RESET': '\033[0m'      # Reset
    }
    
    # Component colors
    COMPONENT_COLORS = {
        'GSTREAMER': '\033[94m',   # Blue
        'PIPELINE': '\033[95m',    # Magenta
        'POLLING': '\033[96m',     # Light Cyan
        'WEBSOCKET': '\033[93m',   # Light Yellow
        'CAMERA': '\033[92m',      # Light Green
        'MONITOR': '\033[97m',     # White
        'ARUCO': '\033[91m',       # Light Red
        'ERROR': '\033[31m',       # Red
        'QUEUE': '\033[90m',       # Dark Gray
        'STATS': '\033[37m',       # Light Gray
        'RESET': '\033[0m'         # Reset
    }
    
    def format(self, record):
        # Add color to log level
        level_color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        
        # Check for component tags in the message
        message = record.getMessage()
        for component, color in self.COMPONENT_COLORS.items():
            if f'[{component}]' in message:
                message = message.replace(f'[{component}]', f'{color}[{component}]{self.COLORS["RESET"]}')
                break
        
        # Format the record
        formatted = super().format(record)
        return f"{level_color}{formatted}{self.COLORS['RESET']}"

# Configure logging with colored formatter
handler = logging.StreamHandler()
handler.setFormatter(ColoredFormatter('%(asctime)s - %(levelname)s - %(message)s'))
logger = logging.getLogger(__name__)
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

class GStreamerCameraReader:
    """
    GStreamer-based camera reader that receives RTP/H.264 streams via UDP
    using GStreamer Python bindings (gi) for reliable UDP stream handling.
    """
    
    def __init__(self, port: int, camera_id: str = None):
        """
        Initialize GStreamer camera reader.
        
        Args:
            port: UDP port to listen on for RTP stream
            camera_id: Optional camera identifier for logging
        """
        self.port = port
        self.camera_id = camera_id or f"camera_{port}"
        self.pipeline = None
        self.appsink = None
        self.loop = None
        self.loop_thread = None
        self.is_opened = False
        self.running = False
        
        # Frame queue for thread-safe frame access
        self.frame_queue = Queue(maxsize=5)  # Increased from 2 to 5 frames
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Frame statistics
        self.frame_count = 0
        self.last_frame_time = 0
        self.start_time = 0
        
        # Frame dimensions
        self.width = 0
        self.height = 0
        
        logger.info(f"Initializing GStreamer reader for {self.camera_id} on port {port}")
        
        # Try to open the pipeline
        self._open_pipeline()
    
    def _build_pipeline_string(self, port: int) -> str:
        """
        Build GStreamer pipeline string for RTP/H.264 reception.
        """
        config = get_backend_config()
        gst_config = config["GSTREAMER_CONFIG"]
        
        elements = gst_config['pipeline_elements']
        logger.debug(f"Pipeline elements for {self.camera_id}: {elements}")
        
        # Proper pipeline with one udpsrc followed by all elements
        udpsrc = f"udpsrc port={port} caps=\"{gst_config['rtp_caps']}\""
        # Add name to appsink element so we can find it later
        elements_with_name = []
        for element in elements:
            if element.startswith("appsink"):
                elements_with_name.append("appsink name=sink drop=true max-buffers=10")  # Increased to 10 as suggested
            else:
                elements_with_name.append(element)
        
        # Add queue elements for better buffering - comprehensive queue approach
        pipeline_elements = []
        pipeline_elements.append(udpsrc)
        
        # Add queue after udpsrc
        pipeline_elements.append("queue max-size-buffers=30 max-size-bytes=0 max-size-time=0")
        
        # Add each element with a queue before it (except the first one)
        for i, element in enumerate(elements_with_name):
            if element.startswith("appsink"):
                # Add queue before appsink
                pipeline_elements.append("queue max-size-buffers=10 max-size-bytes=0 max-size-time=0")
                pipeline_elements.append(element)
            else:
                pipeline_elements.append(element)
                # Add queue after each element (except appsink)
                pipeline_elements.append("queue max-size-buffers=15 max-size-bytes=0 max-size-time=0")
        
        pipeline = " ! ".join(pipeline_elements)
        
        logger.info(f"[{self.camera_id}] [PIPELINE] Enhanced pipeline with comprehensive buffering:")
        logger.info(f"[{self.camera_id}] [PIPELINE] {pipeline}")
        return pipeline
    
    def _on_new_sample(self, sink):
        """Callback for new frame samples from GStreamer appsink."""
        try:
            sample = sink.emit("pull-sample")
            if sample:
                buffer = sample.get_buffer()
                caps = sample.get_caps()
                
                # Get buffer info
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if success:
                    frame_data = map_info.data
                    buffer.unmap(map_info)
                    
                    # Convert to numpy array
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    
                    # Get frame dimensions from caps
                    width = caps.get_structure(0).get_value("width")
                    height = caps.get_structure(0).get_value("height")
                    
                    # Reshape frame to 2D array (assuming BGR format)
                    if width and height:
                        frame = frame.reshape((height, width, 3))
                        
                        # Update frame statistics
                        self.frame_count += 1
                        self.last_frame_time = time.time()
                        
                        # Initialize start_time on first frame
                        if self.start_time == 0:
                            self.start_time = time.time()
                        
                        logger.info(f"[{self.camera_id}] [GSTREAMER] Processed frame {self.frame_count}, size: {frame.shape}, buffer size: {len(frame_data)} bytes")
                        
                        # Store frame in queue and as latest frame
                        try:
                            if not self.frame_queue.full():
                                self.frame_queue.put_nowait(frame)
                        except:
                            pass  # Queue is full, skip this frame
                        
                        with self.frame_lock:
                            self.latest_frame = frame
                        
                        return Gst.FlowReturn.OK
                    else:
                        logger.warning(f"[{self.camera_id}] [GSTREAMER] Invalid frame dimensions: {width}x{height}")
                        return Gst.FlowReturn.ERROR
                else:
                    logger.error(f"[{self.camera_id}] [GSTREAMER] Failed to map buffer")
                    return Gst.FlowReturn.ERROR
            else:
                logger.warning(f"[{self.camera_id}] [GSTREAMER] No sample available")
                return Gst.FlowReturn.ERROR
        except Exception as e:
            logger.error(f"[{self.camera_id}] [GSTREAMER] Error in new-sample callback: {e}")
            return Gst.FlowReturn.ERROR
    
    def _run_glib_loop(self):
        """Run GLib main loop in separate thread."""
        try:
            self.loop = GLib.MainLoop()
            self.running = True
            logger.debug(f"Starting GLib main loop for {self.camera_id}")
            self.loop.run()
        except Exception as e:
            logger.error(f"GLib main loop error for {self.camera_id}: {e}")
        finally:
            self.running = False
            logger.debug(f"GLib main loop ended for {self.camera_id}")
    
    def _on_bus_message(self, bus, message):
        """Handle GStreamer bus messages."""
        t = message.type
        
        if t == Gst.MessageType.EOS:
            logger.info(f"End of stream for {self.camera_id}")
            self.stop()
            
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error for {self.camera_id}: {err} ({debug})")
            self.stop()
            
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"GStreamer warning for {self.camera_id}: {err} ({debug})")
            
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                logger.debug(f"Pipeline state changed for {self.camera_id}: {old_state.value_name} -> {new_state.value_name}")
                
        return True
    
    def stop(self):
        """Stop the GStreamer pipeline and clean up resources."""
        self.running = False
        
        if self.pipeline:
            logger.info(f"[{self.camera_id}] [PIPELINE] Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        
        if self.loop:
            logger.info(f"[{self.camera_id}] [PIPELINE] Stopping GLib loop...")
            self.loop.quit()
            self.loop = None
        
        self.is_opened = False
        logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline stopped")
    
    def _open_pipeline(self):
        """Open the GStreamer pipeline using gi bindings."""
        try:
            # Build pipeline string
            pipeline_str = self._build_pipeline_string(self.port)
            logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline string: {pipeline_str}")
            
            # Create pipeline
            self.pipeline = Gst.parse_launch(pipeline_str)
            if self.pipeline is None:
                raise RuntimeError(f"Failed to create GStreamer pipeline for {self.camera_id}")
            
            logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline created successfully")
            
            # Get appsink element
            self.appsink = self.pipeline.get_by_name("sink")
            if self.appsink is None:
                raise RuntimeError(f"Failed to get appsink element for {self.camera_id}")
            
            logger.info(f"[{self.camera_id}] [PIPELINE] Appsink element found")
            
            # Configure appsink properties
            self.appsink.set_property("emit-signals", True)
            self.appsink.set_property("drop", True)
            self.appsink.set_property("max-buffers", 10)
            self.appsink.set_property("sync", False)  # Important: set to False for real-time streaming
            
            # Connect new-sample signal
            self.appsink.connect("new-sample", self._on_new_sample)
            logger.info(f"[{self.camera_id}] [PIPELINE] Connected new-sample signal")
            
            # Connect bus message handler
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)
            logger.info(f"[{self.camera_id}] [PIPELINE] Connected bus message handler")
            
            # Start GLib loop in separate thread
            self.loop_thread = threading.Thread(target=self._run_glib_loop, daemon=True)
            self.loop_thread.start()
            
            # Start pipeline
            logger.info(f"[{self.camera_id}] [PIPELINE] Starting pipeline...")
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError(f"Failed to start pipeline for {self.camera_id}")
            
            logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline state change initiated: {ret}")
            
            # Wait for pipeline to reach PLAYING state
            logger.info(f"[{self.camera_id}] [PIPELINE] Waiting for pipeline to reach PLAYING state...")
            ret = self.pipeline.get_state(timeout=Gst.CLOCK_TIME_NONE)
            logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline state: {ret[0]}, ret: {ret[1]}, pending: {ret[2]}")
            
            if ret[0] != Gst.StateChangeReturn.SUCCESS:
                raise RuntimeError(f"Pipeline failed to reach PLAYING state for {self.camera_id}: {ret[0]}")
            
            logger.info(f"[{self.camera_id}] [PIPELINE] GStreamer pipeline opened successfully on port {self.port}")
            self.is_opened = True
            
        except Exception as e:
            logger.error(f"[{self.camera_id}] [PIPELINE] Error opening pipeline: {e}")
            self.stop()
            raise
    
    def _monitor_pipeline_state(self):
        """Monitor pipeline state and log any issues."""
        def monitor():
            while self.running and self.is_opened:
                try:
                    ret, state, pending = self.pipeline.get_state(0)  # Non-blocking
                    if ret == Gst.StateChangeReturn.FAILURE:
                        logger.error(f"[{self.camera_id}] [MONITOR] Pipeline state change failed during monitoring")
                        break
                    elif state != Gst.State.PLAYING:
                        logger.warning(f"[{self.camera_id}] [MONITOR] Pipeline not in PLAYING state: {state}")
                    
                    # Check for bus messages
                    bus = self.pipeline.get_bus()
                    if bus:
                        msg = bus.pop()
                        if msg:
                            if msg.type == Gst.MessageType.ERROR:
                                err, debug = msg.parse_error()
                                logger.error(f"[{self.camera_id}] [MONITOR] GStreamer error: {err}, debug: {debug}")
                            elif msg.type == Gst.MessageType.WARNING:
                                warn, debug = msg.parse_warning()
                                logger.warning(f"[{self.camera_id}] [MONITOR] GStreamer warning: {warn}, debug: {debug}")
                            elif msg.type == Gst.MessageType.EOS:
                                logger.info(f"[{self.camera_id}] [MONITOR] End of stream")
                    
                    time.sleep(1)  # Check every second
                except Exception as e:
                    logger.error(f"[{self.camera_id}] [MONITOR] Error in pipeline monitoring: {e}")
                    time.sleep(1)
        
        # Start monitoring in separate thread
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
        logger.info(f"[{self.camera_id}] [MONITOR] Pipeline monitoring started")
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        Read the latest frame from the GStreamer pipeline.
        
        Returns:
            OpenCV image array or None if no frame available
        """
        if not self.is_opened:
            return None
        
        try:
            # Try to get frame from queue first (most recent)
            try:
                frame = self.frame_queue.get_nowait()
                logger.debug(f"[{self.camera_id}] [READ] Got frame from queue, shape: {frame.shape}")
                return frame
            except Empty:
                logger.debug(f"[{self.camera_id}] [READ] No frame in queue")
                pass
            
            # Fall back to latest stored frame
            with self.frame_lock:
                if self.latest_frame is not None:
                    logger.debug(f"[{self.camera_id}] [READ] Got frame from latest_frame, shape: {self.latest_frame.shape}")
                    return self.latest_frame.copy()
                else:
                    logger.debug(f"[{self.camera_id}] [READ] No latest_frame available")
            
            return None
            
        except Exception as e:
            logger.error(f"Error reading frame from {self.camera_id}: {e}")
            return None
    
    def is_active(self) -> bool:
        """
        Check if the camera reader is active and receiving frames.
        
        Returns:
            True if pipeline is open and receiving frames
        """
        if not self.is_opened or not self.running:
            return False
        
        # Check if we've received frames recently (within last 5 seconds)
        if self.last_frame_time > 0:
            return (time.time() - self.last_frame_time) < 5.0
        
        return False
    
    def get_frame_rate(self) -> float:
        """
        Get the estimated frame rate of the stream.
        
        Returns:
            Frame rate in FPS, or 0 if not available
        """
        if not self.is_active():
            return 0.0
        
        # Calculate actual FPS based on frame count and elapsed time
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0 and self.frame_count > 10:
            return self.frame_count / elapsed_time
        
        return 0.0
    
    def get_resolution(self) -> tuple:
        """
        Get the resolution of the stream.
        
        Returns:
            Tuple of (width, height), or (0, 0) if not available
        """
        if not self.is_active():
            return (0, 0)
        
        return (self.width, self.height)
    
    def _cleanup(self):
        """Internal cleanup method."""
        try:
            # Stop pipeline
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
            
            # Stop GLib loop
            if self.loop is not None:
                self.loop.quit()
            
            # Wait for loop thread to finish
            if self.loop_thread is not None and self.loop_thread.is_alive():
                self.loop_thread.join(timeout=2.0)
            
            # Clear references
            self.pipeline = None
            self.appsink = None
            self.loop = None
            self.loop_thread = None
            
            # Clear frame data
            with self.frame_lock:
                self.latest_frame = None
            
            # Clear frame queue
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    break
            
        except Exception as e:
            logger.error(f"Error during cleanup for {self.camera_id}: {e}")
    
    def release(self):
        """Release the GStreamer pipeline and cleanup resources."""
        try:
            self.running = False
            self.is_opened = False
            
            self._cleanup()
            
            logger.info(f"Released GStreamer pipeline for {self.camera_id}")
            
        except Exception as e:
            logger.error(f"Error releasing GStreamer pipeline for {self.camera_id}: {e}")
    
    def restart(self):
        """Restart the GStreamer pipeline."""
        logger.info(f"Restarting GStreamer pipeline for {self.camera_id}")
        self.release()
        time.sleep(2)  # Longer pause for UDP streams
        self._open_pipeline()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.release()
    
    def __del__(self):
        """Destructor to ensure cleanup."""
        self.release()


class MultiGStreamerReader:
    """
    Manager for multiple GStreamer camera readers.
    """
    
    def __init__(self):
        self.readers: Dict[str, GStreamerCameraReader] = {}
        self.running = False
    
    def add_camera(self, camera_id: str, port: int) -> GStreamerCameraReader:
        """
        Add a new camera reader.
        
        Args:
            camera_id: Unique camera identifier
            port: UDP port for the camera stream
            
        Returns:
            GStreamerCameraReader instance
        """
        if camera_id in self.readers:
            logger.warning(f"Camera {camera_id} already exists, replacing...")
            self.remove_camera(camera_id)
        
        try:
            reader = GStreamerCameraReader(port, camera_id)
            self.readers[camera_id] = reader
            logger.info(f"Added camera reader for {camera_id} on port {port}")
            return reader
        except Exception as e:
            logger.error(f"Failed to add camera reader for {camera_id}: {e}")
            raise
    
    def remove_camera(self, camera_id: str):
        """
        Remove a camera reader.
        
        Args:
            camera_id: Camera identifier to remove
        """
        if camera_id in self.readers:
            self.readers[camera_id].release()
            del self.readers[camera_id]
            logger.info(f"Removed camera reader for {camera_id}")
    
    def get_reader(self, camera_id: str) -> Optional[GStreamerCameraReader]:
        """
        Get a camera reader by ID.
        
        Args:
            camera_id: Camera identifier
            
        Returns:
            GStreamerCameraReader instance or None if not found
        """
        return self.readers.get(camera_id)
    
    def get_all_readers(self) -> Dict[str, GStreamerCameraReader]:
        """
        Get all camera readers.
        
        Returns:
            Dictionary of camera_id -> GStreamerCameraReader
        """
        return self.readers.copy()
    
    def get_active_cameras(self) -> list:
        """
        Get list of active camera IDs.
        
        Returns:
            List of camera IDs that are currently active
        """
        return [cam_id for cam_id, reader in self.readers.items() if reader.is_active()]
    
    def release_all(self):
        """Release all camera readers."""
        for camera_id in list(self.readers.keys()):
            self.remove_camera(camera_id)
        logger.info("Released all camera readers")
    
    def restart_camera(self, camera_id: str):
        """
        Restart a specific camera reader.
        
        Args:
            camera_id: Camera identifier to restart
        """
        if camera_id in self.readers:
            self.readers[camera_id].restart()
            logger.info(f"Restarted camera reader for {camera_id}")
        else:
            logger.warning(f"Camera {camera_id} not found for restart")


if __name__ == "__main__":
    import sys
    
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
        duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        
        logger.info(f"Testing GStreamer reader on port {port} for {duration} seconds")
        
        try:
            with GStreamerCameraReader(port, f"test_camera_{port}") as reader:
                if not reader.is_active():
                    logger.error("Failed to activate GStreamer reader")
                    sys.exit(1)
                
                logger.info(f"Resolution: {reader.get_resolution()}")
                logger.info(f"Frame rate: {reader.get_frame_rate()}")
                
                start_time = time.time()
                frame_count = 0
                
                while time.time() - start_time < duration:
                    frame = reader.read_frame()
                    if frame is not None:
                        frame_count += 1
                        if frame_count % 30 == 0:  # Log every 30 frames
                            logger.info(f"Received {frame_count} frames")
                    else:
                        time.sleep(0.033)  # ~30 FPS
                
                logger.info(f"Test completed. Received {frame_count} frames in {duration} seconds")
                
        except Exception as e:
            logger.error(f"Test failed: {e}")
            sys.exit(1)
    else:
        print("Usage: python3 gstreamer_reader.py <port> [duration]")
        print("Example: python3 gstreamer_reader.py 5004 10")
        sys.exit(1)