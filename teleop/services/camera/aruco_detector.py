#!/usr/bin/env python3
"""
ArUco marker detection module for multi-camera streaming system.
Processes H.264 frames and overlays detected ArUco markers.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List
import base64
import io

logger = logging.getLogger(__name__)

class ArucoDetector:
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_100):
        """
        Initialize ArUco detector.
        
        Args:
            dictionary_type: ArUco dictionary type (default: DICT_4X4_100)
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Detection settings
        self.marker_border_color = (0, 255, 0)  # Green
        self.marker_border_thickness = 2
        self.text_color = (255, 255, 255)  # White
        self.text_thickness = 2
        self.text_font = cv2.FONT_HERSHEY_SIMPLEX
        self.text_scale = 0.7
        
        logger.info(f"ArUco detector initialized with dictionary type: {dictionary_type}")
    
    def decode_h264_frame(self, h264_data: bytes) -> Optional[np.ndarray]:
        """
        Decode H.264 frame data to OpenCV image.
        
        Args:
            h264_data: Raw H.264 frame bytes
            
        Returns:
            OpenCV image array or None if decoding fails
        """
        try:
            # Create a temporary file-like object
            buffer = io.BytesIO(h264_data)
            
            # Use OpenCV to decode H.264 data
            # Note: This is a simplified approach. For production, you might want
            # to use a more robust H.264 decoder like FFmpeg
            
            # Try to decode as if it's already a JPEG (fallback)
            try:
                # Convert to numpy array
                nparr = np.frombuffer(h264_data, np.uint8)
                # Try to decode as image
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is not None:
                    return img
            except Exception:
                pass
            
            # If direct decoding fails, we need a proper H.264 decoder
            # For now, return None to indicate decoding failure
            logger.warning("H.264 decoding not implemented - frame skipped")
            return None
            
        except Exception as e:
            logger.error(f"Failed to decode H.264 frame: {e}")
            return None
    
    def detect_markers(self, image: np.ndarray) -> Tuple[List, List, np.ndarray]:
        """
        Detect ArUco markers in image.
        
        Args:
            image: OpenCV image array
            
        Returns:
            Tuple of (corners, ids, processed_image)
        """
        try:
            # Convert to grayscale for detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            
            # Draw detected markers on the image
            processed_image = image.copy()
            if ids is not None and len(ids) > 0:
                # Draw marker borders
                cv2.aruco.drawDetectedMarkers(
                    processed_image, corners, ids,
                    borderColor=self.marker_border_color
                )
                
                # Add marker ID labels
                for i, marker_id in enumerate(ids.flatten()):
                    # Get marker center
                    corner = corners[i][0]
                    center_x = int(np.mean(corner[:, 0]))
                    center_y = int(np.mean(corner[:, 1]))
                    
                    # Draw marker ID text
                    text = f"ID: {marker_id}"
                    text_size = cv2.getTextSize(text, self.text_font, self.text_scale, self.text_thickness)[0]
                    
                    # Position text above marker
                    text_x = center_x - text_size[0] // 2
                    text_y = center_y - 10
                    
                    # Draw text background
                    cv2.rectangle(
                        processed_image,
                        (text_x - 5, text_y - text_size[1] - 5),
                        (text_x + text_size[0] + 5, text_y + 5),
                        (0, 0, 0),  # Black background
                        -1
                    )
                    
                    # Draw text
                    cv2.putText(
                        processed_image, text,
                        (text_x, text_y),
                        self.text_font, self.text_scale,
                        self.text_color, self.text_thickness
                    )
                
                logger.debug(f"Detected {len(ids)} ArUco markers: {ids.flatten().tolist()}")
            
            return corners, ids, processed_image
            
        except Exception as e:
            logger.error(f"Failed to detect ArUco markers: {e}")
            return [], [], image
    
    def process_frame(self, frame_data: bytes, is_h264: bool = True) -> Optional[bytes]:
        """
        Process frame data and return frame with ArUco markers overlaid.
        
        Args:
            frame_data: Raw frame bytes (H.264 or JPEG)
            is_h264: Whether frame data is H.264 encoded
            
        Returns:
            Processed frame as JPEG bytes or None if processing fails
        """
        try:
            if is_h264:
                # Decode H.264 frame
                image = self.decode_h264_frame(frame_data)
                if image is None:
                    # If H.264 decoding fails, return original data
                    return frame_data
            else:
                # Decode JPEG frame
                nparr = np.frombuffer(frame_data, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if image is None:
                    logger.error("Failed to decode JPEG frame")
                    return frame_data
            
            # Detect and draw ArUco markers
            corners, ids, processed_image = self.detect_markers(image)
            
            # Encode processed image back to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
            result, encoded_img = cv2.imencode('.jpg', processed_image, encode_param)
            
            if result:
                return encoded_img.tobytes()
            else:
                logger.error("Failed to encode processed image")
                return frame_data
                
        except Exception as e:
            logger.error(f"Failed to process frame: {e}")
            return frame_data
    
    def process_frame_base64(self, frame_b64: str, is_h264: bool = True) -> str:
        """
        Process base64-encoded frame and return processed frame as base64.
        
        Args:
            frame_b64: Base64-encoded frame data
            is_h264: Whether frame data is H.264 encoded
            
        Returns:
            Processed frame as base64 string
        """
        try:
            # Decode base64
            frame_data = base64.b64decode(frame_b64)
            
            # Process frame
            processed_data = self.process_frame(frame_data, is_h264)
            
            if processed_data:
                # Encode back to base64
                return base64.b64encode(processed_data).decode('utf-8')
            else:
                return frame_b64
                
        except Exception as e:
            logger.error(f"Failed to process base64 frame: {e}")
            return frame_b64

class H264Decoder:
    """
    Proper H.264 decoder using OpenCV's VideoCapture with memory buffer.
    This is a more robust solution for H.264 decoding.
    """
    
    def __init__(self):
        self.temp_file_counter = 0
    
    def decode_frame(self, h264_data: bytes) -> Optional[np.ndarray]:
        """
        Decode H.264 frame using temporary file approach.
        
        Args:
            h264_data: Raw H.264 frame bytes
            
        Returns:
            OpenCV image array or None if decoding fails
        """
        try:
            import tempfile
            import os
            
            # Create temporary file
            with tempfile.NamedTemporaryFile(suffix='.h264', delete=False) as temp_file:
                temp_file.write(h264_data)
                temp_filename = temp_file.name
            
            try:
                # Use ffmpeg to convert H.264 to image
                import subprocess
                
                # Convert H.264 to PNG using ffmpeg
                png_filename = temp_filename.replace('.h264', '.png')
                cmd = [
                    'ffmpeg', '-y', '-i', temp_filename,
                    '-vframes', '1', '-f', 'image2', png_filename
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                if result.returncode == 0 and os.path.exists(png_filename):
                    # Read the converted image
                    image = cv2.imread(png_filename)
                    os.unlink(png_filename)  # Clean up
                    return image
                else:
                    logger.warning(f"FFmpeg conversion failed: {result.stderr}")
                    return None
                    
            finally:
                # Clean up temporary file
                if os.path.exists(temp_filename):
                    os.unlink(temp_filename)
                    
        except Exception as e:
            logger.error(f"H.264 decoding failed: {e}")
            return None

# Factory function to create detector with different configurations
def create_aruco_detector(dictionary_name: str = "DICT_4X4_100") -> ArucoDetector:
    """
    Create ArUco detector with specified dictionary.
    
    Args:
        dictionary_name: Name of ArUco dictionary
        
    Returns:
        ArucoDetector instance
    """
    dictionary_map = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    }
    
    dictionary_type = dictionary_map.get(dictionary_name, cv2.aruco.DICT_4X4_100)
    return ArucoDetector(dictionary_type)

if __name__ == "__main__":
    # Test the ArUco detector
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    detector = create_aruco_detector()
    
    # Test with a sample image if provided
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
        image = cv2.imread(image_path)
        if image is not None:
            corners, ids, processed_image = detector.detect_markers(image)
            
            if ids is not None:
                print(f"Detected markers: {ids.flatten().tolist()}")
                cv2.imshow("ArUco Detection", processed_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print("No ArUco markers detected")
        else:
            print(f"Failed to load image: {image_path}")
    else:
        print("ArUco detector module loaded successfully")
        print("Usage: python3 aruco_detector.py <image_path>")
