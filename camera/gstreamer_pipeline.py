#!/usr/bin/env python3
"""
GStreamer pipeline manager using Python bindings (gi).
Handles pipeline creation, state management, and dynamic updates.
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import logging
from typing import Optional, List
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
    intermediate_elements: List[Gst.Element]
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

        # Initialize settings (clamped to modest defaults to reduce buffer pressure)
        self.width = min(self.config.get('capture_width', 640), 640)
        self.height = min(self.config.get('capture_height', 480), 480)
        self.framerate = min(self.config.get('default_fps', 20), 30)
        self.bitrate = self.gst_config.get('h264_bitrate', 512)
        self.h264_tune = self.gst_config.get('h264_tune', 'zerolatency')

        # Pipeline state
        self.elements: Optional[PipelineElements] = None
        self.mainloop: Optional[GLib.MainLoop] = None
        self.mainloop_thread: Optional[threading.Thread] = None
        self.is_running = False
        self.camera_type = None

        # Optional: external frame callback
        self.frame_callback = None

    # -------------------------
    # Pipeline construction
    # -------------------------
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

            # Source
            src = Gst.ElementFactory.make('v4l2src')
            if not src:
                raise RuntimeError("Failed to create v4l2src element")
            src.set_property('device', self.device_path)

            # Caps filter (flexible to let negotiation pick supported mode)
            capsfilter = Gst.ElementFactory.make('capsfilter')
            if not capsfilter:
                raise RuntimeError("Failed to create capsfilter element")

            self.camera_type = self._detect_camera_format()
            caps_str = self._get_flexible_format_caps()
            caps = Gst.Caps.from_string(caps_str)
            capsfilter.set_property('caps', caps)

            # Small queue to decouple capture from downstream backpressure
            queue = Gst.ElementFactory.make('queue')
            if not queue:
                raise RuntimeError("Failed to create queue element")
            queue.set_property('max-size-buffers', 4)
            queue.set_property('leaky', 2)  # downstream

            # Intermediate elements
            intermediate_elements: List[Gst.Element] = []

            if self.camera_type == 'mjpg':
                # MJPEG needs a JPEG decoder before colorspace conversion
                jpegdec = Gst.ElementFactory.make('jpegdec')
                if not jpegdec:
                    raise RuntimeError("Failed to create jpegdec element")
                intermediate_elements.append(jpegdec)

            converter = Gst.ElementFactory.make('videoconvert')
            if not converter:
                raise RuntimeError("Failed to create videoconvert element")
            intermediate_elements.append(converter)

            # Normalize to NV12 for encoders that prefer it
            nv12_capsfilter = Gst.ElementFactory.make('capsfilter')
            if not nv12_capsfilter:
                raise RuntimeError("Failed to create NV12 capsfilter element")
            nv12_caps = Gst.Caps.from_string("video/x-raw,format=NV12")
            nv12_capsfilter.set_property('caps', nv12_caps)
            intermediate_elements.append(nv12_capsfilter)

            # Encoder
            encoder = Gst.ElementFactory.make('x264enc')
            if not encoder:
                raise RuntimeError("Failed to create x264enc element")
            encoder.set_property('tune', self.h264_tune)
            encoder.set_property('bitrate', self.bitrate)

            # RTP payloader
            payloader = Gst.ElementFactory.make('rtph264pay')
            if not payloader:
                raise RuntimeError("Failed to create rtph264pay element")
            payloader.set_property('config-interval', 1)
            payloader.set_property('pt', 96)

            # UDP sink
            sink = Gst.ElementFactory.make('udpsink')
            if not sink:
                raise RuntimeError("Failed to create udpsink element")
            sink.set_property('host', self.host)
            sink.set_property('port', self.port)

            # Add in order
            elements = [src, capsfilter, queue] + intermediate_elements + [encoder, payloader, sink]
            for element in elements:
                pipeline.add(element)

            # Link chain
            if not src.link(capsfilter):
                raise RuntimeError("Failed to link src -> capsfilter")
            if not capsfilter.link(queue):
                raise RuntimeError("Failed to link capsfilter -> queue")

            current = queue
            for el in intermediate_elements:
                if not current.link(el):
                    raise RuntimeError(f"Failed to link {current.get_name()} -> {el.get_name()}")
                current = el

            if not current.link(encoder):
                raise RuntimeError(f"Failed to link {current.get_name()} -> encoder")
            if not encoder.link(payloader):
                raise RuntimeError("Failed to link encoder -> payloader")
            if not payloader.link(sink):
                raise RuntimeError("Failed to link payloader -> sink")

            bus = pipeline.get_bus()
            # IMPORTANT: do NOT add_signal_watch() here; we attach it in start()
            # to ensure it's bound to the same thread/context as the GLib.MainLoop.

            self.elements = PipelineElements(
                pipeline=pipeline,
                src=src,
                capsfilter=capsfilter,
                intermediate_elements=[queue] + intermediate_elements,
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

    # -------------------------
    # Format helpers
    # -------------------------
    def _detect_camera_format(self) -> str:
        """
        Detect camera format using v4l2-ctl.

        Returns:
            Camera format type ('mjpg', 'yuyv', or 'raw')
        """
        import subprocess
        try:
            output = subprocess.check_output(
                ['v4l2-ctl', '--device', self.device_path, '--list-formats-ext'],
                text=True
            )
            if 'MJPG' in output:
                return 'mjpg'
            elif 'YUYV' in output or 'YUYV8' in output:
                return 'yuyv'
            else:
                return 'raw'
        except Exception as e:
            logger.warning(f"Failed to detect camera format for {self.device_path}: {e}")
            return 'raw'

    def _get_format_caps(self) -> str:
        """
        Get GStreamer caps string for detected format from config (fixed).
        Not used in the flexible pipeline but kept for completeness.
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

    def _get_flexible_format_caps(self) -> str:
        """
        Get flexible GStreamer caps string that allows negotiation.
        This avoids forcing unsupported formats at the source.
        """
        if self.camera_type == 'mjpg':
            return "image/jpeg"
        elif self.camera_type == 'yuyv':
            return "video/x-raw,format=YUY2"
        else:
            return "video/x-raw"

    # -------------------------
    # Bus handling / mainloop
    # -------------------------
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
            # Guard in case elements have been cleared during teardown
            if self.elements and message.src == self.elements.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                logger.debug(
                    f"Pipeline state changed for {self.camera_id}: "
                    f"{old_state.value_name} -> {new_state.value_name}"
                )

    def _run_mainloop(self):
        """Run GLib main loop in separate thread."""
        try:
            self.mainloop.run()
        except Exception as e:
            logger.error(f"Error in main loop for {self.camera_id}: {e}")
        finally:
            self.is_running = False

    # -------------------------
    # Lifecycle
    # -------------------------
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

            # Connect and watch bus **now**, so the watch belongs to the same
            # thread/context as the GLib.MainLoop
            self.elements.bus.connect('message', self._on_bus_message)
            self.elements.bus.add_signal_watch()

            # Set pipeline state to PLAYING
            ret = self.elements.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to PLAYING state")

            # Start main loop in separate thread
            self.is_running = True
            self.mainloop_thread = threading.Thread(target=self._run_mainloop, daemon=True)
            self.mainloop_thread.start()

            logger.info(f"Started GStreamer pipeline for {self.camera_id}")
            return True

        except Exception as e:
            logger.error(f"Failed to start pipeline for {self.camera_id}: {e}")
            self.stop()
            return False

    def stop(self):
        """Stop the GStreamer pipeline with proper threading cleanup."""
        try:
            self.is_running = False

            if self.elements and self.elements.pipeline:
                logger.info(f"[{self.camera_id}] [PIPELINE] Stopping pipeline...")

                # 1) Try graceful EOS first
                try:
                    self.elements.pipeline.send_event(Gst.Event.new_eos())
                    # Wait up to ~1s for EOS or ERROR to arrive
                    self.elements.pipeline.get_bus().timed_pop_filtered(
                        1_000_000_000,  # 1s in ns
                        Gst.MessageType.EOS | Gst.MessageType.ERROR
                    )
                except Exception:
                    pass

                # 2) Force NULL and wait for the state change to complete
                self.elements.pipeline.set_state(Gst.State.NULL)
                try:
                    self.elements.pipeline.get_state(Gst.SECOND * 2)  # wait up to 2s
                except Exception:
                    pass

            # 3) Stop the GLib loop
            if self.mainloop:
                logger.info(f"[{self.camera_id}] [PIPELINE] Stopping GLib loop...")
                self.mainloop.quit()

            # 4) Wait for main loop thread to finish (with timeout)
            if self.mainloop_thread and self.mainloop_thread.is_alive():
                if self.mainloop_thread != threading.current_thread():
                    self.mainloop_thread.join(timeout=2.0)

            # 5) Remove bus watch (if still present)
            if self.elements and self.elements.bus:
                try:
                    self.elements.bus.remove_signal_watch()
                except Exception:
                    pass

            # 6) Clear references to allow GC to unref objects (and close /dev/videoX)
            self.mainloop = None
            self.mainloop_thread = None
            self.elements = None

            logger.info(f"[{self.camera_id}] [PIPELINE] Pipeline stopped")

        except Exception as e:
            logger.error(f"[{self.camera_id}] [PIPELINE] Error stopping pipeline: {e}")

    # -------------------------
    # Dynamic updates
    # -------------------------
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
            if not old_caps or old_caps.is_empty():
                logger.warning(f"No caps to update framerate for {self.camera_id}")
                return False

            structure = old_caps.get_structure(0)
            # Try to replace a simple framerate=X/1 pattern if present; otherwise, reapply caps
            old_caps_str = structure.to_string()
            if f"framerate={self.framerate}/1" in old_caps_str:
                new_caps_str = old_caps_str.replace(
                    f"framerate={self.framerate}/1",
                    f"framerate={fps}/1"
                )
            else:
                # If no explicit framerate field exists, add one to the flexible caps
                new_caps_str = old_caps_str + f",framerate={fps}/1"

            new_caps = Gst.Caps.from_string(new_caps_str)
            self.elements.capsfilter.set_property('caps', new_caps)
            self.framerate = fps

            logger.info(f"Updated framerate for {self.camera_id} to {fps} fps")
            return True

        except Exception as e:
            logger.error(f"Failed to update framerate for {self.camera_id}: {e}")
            return False

    # -------------------------
    # Status helpers
    # -------------------------
    def is_active(self) -> bool:
        """Check if pipeline is active and streaming."""
        if not self.elements or not self.is_running:
            return False

        try:
            ret, state, pending = self.elements.pipeline.get_state(0)
            return state == Gst.State.PLAYING
        except Exception:
            return False

    # -------------------------
    # Context manager
    # -------------------------
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
