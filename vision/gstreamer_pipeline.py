#!/usr/bin/env python3
"""
GStreamer pipeline manager using Python bindings (gi).
Handles pipeline creation, state management, and dynamic updates.
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import logging
from typing import Dict, Optional, Tuple, List, Any
from dataclasses import dataclass
import threading

from config import get_jetson_config, validate_camera_settings

# Initialize GStreamer
Gst.init(None)

logger = logging.getLogger(__name__)

@dataclass
class PipelineElements:
    """Container for GStreamer pipeline elements."""
    pipeline: Gst.Pipeline
    src: Gst.Element
    capsfilter: Gst.Element
    converter: Gst.Element
    encoder: Gst.Element
    payloader: Gst.Element
    sink: Gst.Element
    bus: Gst.Bus

class GStreamerPipeline:
    """
    GStreamer pipeline manager using Python bindings.
    Handles pipeline creation, state management, and dynamic updates.
    """
    
    def __init__(self, device_path: str, camera_id: str, host: str, port: int):
        """
        Initialize GStreamer pipeline.
        
        Args:
            device_path: Path to video device (e.g., /dev/video0)
            camera_id: Unique camera identifier
            host: UDP destination host
            port: UDP destination port
        """
        self.device_path = device_path
        self.camera_id = camera_id
        self.host = host
        self.port = port
        
        # Get configuration
        self.config = get_jetson_config()
        self.gst_config = self.config.get('gstreamer_config', {})
        
        # Initialize settings
        self.width = self.config.get('capture_width', 640)
        self.height = self.config.get('capture_height', 480)
        self.framerate = self.config.get('default_fps', 20)
        self.bitrate = self.gst_config.get('h264_bitrate', 512)
        self.h264_tune = self.gst_config.get('h264_tune', 'zerolatency')
        
        # Pipeline state
        self.elements: Optional[PipelineElements] = None
        self.mainloop: Optional[GLib.MainLoop] = None
        self.mainloop_thread: Optional[threading.Thread] = None
        self.is_running = False
        self.camera_type = None
        
        # Frame callback
        self.frame_callback = None
    
    def build_pipeline(self) -> bool:
        """
        Build GStreamer pipeline with Python bindings.
        
        Returns:
            True if pipeline was built successfully
        """
        try:
            # Create pipeline
            pipeline = Gst.Pipeline.new()
            if not pipeline:
                raise RuntimeError("Failed to create pipeline")
            
            # Create elements
            src = Gst.ElementFactory.make('v4l2src')
            if not src:
                raise RuntimeError("Failed to create v4l2src element")
            src.set_property('device', self.device_path)
            
            # Create caps filter
            capsfilter = Gst.ElementFactory.make('capsfilter')
            if not capsfilter:
                raise RuntimeError("Failed to create capsfilter element")
            
            # Detect camera format and set appropriate caps
            self.camera_type = self._detect_camera_format()
            caps_str = self._get_format_caps()
            caps = Gst.Caps.from_string(caps_str)
            capsfilter.set_property('caps', caps)
            
            # Create converter
            converter = Gst.ElementFactory.make('videoconvert')
            if not converter:
                raise RuntimeError("Failed to create videoconvert element")
            
            # Create encoder
            encoder = Gst.ElementFactory.make('x264enc')
            if not encoder:
                raise RuntimeError("Failed to create x264enc element")
            encoder.set_property('tune', self.h264_tune)
            encoder.set_property('bitrate', self.bitrate)
            
            # Create payloader
            payloader = Gst.ElementFactory.make('rtph264pay')
            if not payloader:
                raise RuntimeError("Failed to create rtph264pay element")
            payloader.set_property('config-interval', 1)
            payloader.set_property('pt', 96)
            
            # Create UDP sink
            sink = Gst.ElementFactory.make('udpsink')
            if not sink:
                raise RuntimeError("Failed to create udpsink element")
            sink.set_property('host', self.host)
            sink.set_property('port', self.port)
            
            # Add elements to pipeline
            elements = [src, capsfilter, converter, encoder, payloader, sink]
            for element in elements:
                pipeline.add(element)
            
            # Link elements
            if not Gst.Element.link_many(src, capsfilter, converter, encoder, payloader, sink):
                raise RuntimeError("Failed to link pipeline elements")
            
            # Get pipeline bus
            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self._on_bus_message)
            
            # Store elements
            self.elements = PipelineElements(
                pipeline=pipeline,
                src=src,
                capsfilter=capsfilter,
                converter=converter,
                encoder=encoder,
                payloader=payloader,
                sink=sink,
                bus=bus
            )
            
            logger.info(f"Built GStreamer pipeline for {self.camera_id} ({self.camera_type})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to build pipeline for {self.camera_id}: {e}")
            self.elements = None
            return False
    
    def _detect_camera_format(self) -> str:
        """
        Detect camera format using v4l2-ctl.
        
        Returns:
            Camera format type ('mjpg', 'yuyv', or 'raw')
        """
        import subprocess
        try:
            output = subprocess.check_output([
                'v4l2-ctl',
                '--device', self.device_path,
                '--list-formats-ext'
            ], text=True)
            
            if 'MJPG' in output:
                return 'mjpg'
            elif 'YUYV' in output or 'YUYV8' in output:
                return 'yuyv'
            else:
                return 'raw'
                
        except Exception as e:
            logger.warning(f"Failed to detect camera format: {e}")
            return 'raw'
    
    def _get_format_caps(self) -> str:
        """
        Get GStreamer caps string for detected format.
        
        Returns:
            GStreamer caps string
        """
        formats = self.gst_config.get('pipeline_formats', {})
        format_config = formats.get(self.camera_type, formats.get('raw', {}))
        
        caps = format_config.get('caps', '')
        if self.camera_type == 'raw':
            caps = caps.format(
                width=self.width,
                height=self.height,
                fps=self.framerate
            )
        
        return caps
    
    def _on_bus_message(self, bus: Gst.Bus, message: Gst.Message):
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
            if message.src == self.elements.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                logger.debug(f"Pipeline state changed for {self.camera_id}: {old_state.value_name} -> {new_state.value_name}")
    
    def start(self) -> bool:
        """
        Start the GStreamer pipeline.
        
        Returns:
            True if pipeline started successfully
        """
        if not self.elements:
            if not self.build_pipeline():
                return False
        
        try:
            # Create GLib main loop
            self.mainloop = GLib.MainLoop()
            
            # Set pipeline state to PLAYING
            ret = self.elements.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to PLAYING state")
            
            # Start main loop in separate thread
            self.is_running = True
            self.mainloop_thread = threading.Thread(target=self._run_mainloop)
            self.mainloop_thread.daemon = True
            self.mainloop_thread.start()
            
            logger.info(f"Started GStreamer pipeline for {self.camera_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start pipeline for {self.camera_id}: {e}")
            self.stop()
            return False
    
    def stop(self):
        """Stop the GStreamer pipeline."""
        try:
            self.is_running = False
            
            if self.mainloop:
                self.mainloop.quit()
            
            if self.elements:
                self.elements.pipeline.set_state(Gst.State.NULL)
            
            if self.mainloop_thread and self.mainloop_thread.is_alive():
                self.mainloop_thread.join(timeout=2.0)
            
            self.mainloop = None
            self.mainloop_thread = None
            
            logger.info(f"Stopped GStreamer pipeline for {self.camera_id}")
            
        except Exception as e:
            logger.error(f"Error stopping pipeline for {self.camera_id}: {e}")
    
    def _run_mainloop(self):
        """Run GLib main loop in separate thread."""
        try:
            self.mainloop.run()
        except Exception as e:
            logger.error(f"Error in main loop for {self.camera_id}: {e}")
        finally:
            self.is_running = False
    
    def update_bitrate(self, bitrate: int) -> bool:
        """
        Update H.264 encoder bitrate.
        
        Args:
            bitrate: New bitrate in kbps
            
        Returns:
            True if update was successful
        """
        try:
            # Validate bitrate
            valid, error = validate_camera_settings(bitrate, self.framerate)
            if not valid:
                logger.warning(f"Invalid bitrate for {self.camera_id}: {error}")
                return False
            
            # Update encoder property
            self.elements.encoder.set_property('bitrate', bitrate)
            self.bitrate = bitrate
            
            logger.info(f"Updated bitrate for {self.camera_id} to {bitrate} kbps")
            return True
            
        except Exception as e:
            logger.error(f"Failed to update bitrate for {self.camera_id}: {e}")
            return False
    
    def update_framerate(self, fps: int) -> bool:
        """
        Update video framerate.
        
        Args:
            fps: New framerate
            
        Returns:
            True if update was successful
        """
        try:
            # Validate FPS
            valid, error = validate_camera_settings(self.bitrate, fps)
            if not valid:
                logger.warning(f"Invalid FPS for {self.camera_id}: {error}")
                return False
            
            # Get current caps
            old_caps = self.elements.capsfilter.get_property('caps')
            structure = old_caps.get_structure(0)
            
            # Create new caps with updated framerate
            new_caps_str = structure.to_string().replace(
                f"framerate={self.framerate}/1",
                f"framerate={fps}/1"
            )
            new_caps = Gst.Caps.from_string(new_caps_str)
            
            # Update capsfilter
            self.elements.capsfilter.set_property('caps', new_caps)
            self.framerate = fps
            
            logger.info(f"Updated framerate for {self.camera_id} to {fps} fps")
            return True
            
        except Exception as e:
            logger.error(f"Failed to update framerate for {self.camera_id}: {e}")
            return False
    
    def is_active(self) -> bool:
        """Check if pipeline is active and streaming."""
        if not self.elements or not self.is_running:
            return False
        
        try:
            # Get current state
            ret, state, pending = self.elements.pipeline.get_state(0)
            return state == Gst.State.PLAYING
        except Exception:
            return False
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()

