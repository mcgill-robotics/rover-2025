#!/usr/bin/env python3
"""
GStreamer-based camera reader for receiving RTP/H.264 streams via UDP.
Replaces manual UDP packet parsing with proper GStreamer pipeline decoding.
"""

import cv2
import numpy as np
import logging
import asyncio
import time
from typing import Optional

logger = logging.getLogger(__name__)

class GStreamerCameraReader:
    """
    GStreamer-based camera reader that receives RTP/H.264 streams via UDP
    and provides decoded frames via OpenCV.
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
        self.pipeline = self._build_pipeline(port)
        self.cap = None
        self.is_opened = False
        
        logger.info(f"Initializing GStreamer reader for {self.camera_id} on port {port}")
        
        # Try to open the pipeline
        self._open_pipeline()
    
    def _build_pipeline(self, port: int) -> str:
        """
        Build GStreamer pipeline string for RTP/H.264 reception.
        
        Args:
            port: UDP port to listen on
            
        Returns:
            GStreamer pipeline string
        """
        from config import get_backend_config
        config = get_backend_config()
        gst_config = config["GSTREAMER_CONFIG"]
        
        # Build pipeline from config
        pipeline_elements = " ! ".join(gst_config["PIPELINE_ELEMENTS"])
        max_buffers = gst_config.get("BUFFER_SIZE", 2)
        
        pipeline = (
            f"udpsrc port={port} "
            f"caps=\"{gst_config['RTP_CAPS']}\" ! "
            f"{pipeline_elements.replace('appsink drop=true max-buffers=2', f'appsink drop=true max-buffers={max_buffers}')}"
        )
        
        logger.debug(f"GStreamer pipeline for {self.camera_id}: {pipeline}")
        return pipeline
    
    def _open_pipeline(self):
        """Open the GStreamer pipeline."""
        try:
            self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                logger.error(f"Failed to open GStreamer pipeline for {self.camera_id} on port {self.port}")
                raise RuntimeError(f"GStreamer pipeline failed for {self.camera_id} on port {self.port}")
            else:
                self.is_opened = True
                logger.info(f"GStreamer pipeline opened successfully for {self.camera_id} on port {self.port}")
                
                # Set buffer size to minimize latency
                from config import get_backend_config
                config = get_backend_config()
                buffer_size = config["GSTREAMER_CONFIG"]["BUFFER_SIZE"]
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
                
        except Exception as e:
            logger.error(f"Exception opening GStreamer pipeline for {self.camera_id}: {e}")
            self.is_opened = False
            raise
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        Read a frame from the GStreamer pipeline.
        
        Returns:
            OpenCV image array or None if no frame available
        """
        if not self.is_opened or self.cap is None:
            logger.warning(f"Pipeline not opened for {self.camera_id}")
            return None
        
        try:
            ret, frame = self.cap.read()
            if not ret:
                logger.debug(f"No frame received from GStreamer pipeline for {self.camera_id}")
                return None
            
            return frame
            
        except Exception as e:
            logger.error(f"Error reading frame from {self.camera_id}: {e}")
            return None
    
    async def read_frame_async(self) -> Optional[np.ndarray]:
        """
        Async wrapper for reading frames.
        
        Returns:
            OpenCV image array or None if no frame available
        """
        # Run the blocking read_frame in a thread pool
        loop = asyncio.get_event_loop()
        try:
            frame = await loop.run_in_executor(None, self.read_frame)
            return frame
        except Exception as e:
            logger.error(f"Async frame read failed for {self.camera_id}: {e}")
            return None
    
    def is_active(self) -> bool:
        """
        Check if the camera reader is active and receiving frames.
        
        Returns:
            True if pipeline is open and ready
        """
        return self.is_opened and self.cap is not None and self.cap.isOpened()
    
    def get_frame_rate(self) -> float:
        """
        Get the frame rate of the stream.
        
        Returns:
            Frame rate in FPS, or 0 if not available
        """
        if not self.is_active():
            return 0.0
        
        try:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            return fps if fps > 0 else 30.0  # Default to 30 FPS if not available
        except Exception:
            return 30.0
    
    def get_resolution(self) -> tuple:
        """
        Get the resolution of the stream.
        
        Returns:
            Tuple of (width, height), or (0, 0) if not available
        """
        if not self.is_active():
            return (0, 0)
        
        try:
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return (width, height)
        except Exception:
            return (0, 0)
    
    def release(self):
        """Release the GStreamer pipeline and cleanup resources."""
        try:
            if self.cap is not None:
                self.cap.release()
                logger.info(f"Released GStreamer pipeline for {self.camera_id}")
            
            self.cap = None
            self.is_opened = False
            
        except Exception as e:
            logger.error(f"Error releasing GStreamer pipeline for {self.camera_id}: {e}")
    
    def restart(self):
        """Restart the GStreamer pipeline."""
        logger.info(f"Restarting GStreamer pipeline for {self.camera_id}")
        self.release()
        time.sleep(1)  # Brief pause before restart
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
        self.readers = {}
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
    
    def get_all_readers(self) -> dict:
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


# Test function
def test_gstreamer_reader(port: int = 5004, duration: int = 10):
    """
    Test the GStreamer reader with a specific port.
    
    Args:
        port: UDP port to test
        duration: Test duration in seconds
    """
    logger.info(f"Testing GStreamer reader on port {port} for {duration} seconds")
    
    try:
        with GStreamerCameraReader(port, f"test_camera_{port}") as reader:
            if not reader.is_active():
                logger.error("Failed to activate GStreamer reader")
                return
            
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
                    
                    # Optional: Display frame (comment out for headless testing)
                    # cv2.imshow(f"Test Camera {port}", frame)
                    # if cv2.waitKey(1) & 0xFF == ord('q'):
                    #     break
                else:
                    time.sleep(0.033)  # ~30 FPS
            
            logger.info(f"Test completed. Received {frame_count} frames in {duration} seconds")
            # cv2.destroyAllWindows()
            
    except Exception as e:
        logger.error(f"Test failed: {e}")


if __name__ == "__main__":
    import sys
    
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
        duration = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        test_gstreamer_reader(port, duration)
    else:
        print("Usage: python3 gstreamer_reader.py <port> [duration]")
        print("Example: python3 gstreamer_reader.py 5004 10")
