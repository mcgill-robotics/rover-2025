#!/usr/bin/env python3
"""
ArUco marker detection module for multi-camera streaming system.
Processes OpenCV frames and overlays detected ArUco markers.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List
import base64

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
    
    def process_frame(self, image: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Process OpenCV frame and return frame with ArUco markers overlaid.
        
        Args:
            image: OpenCV image array (BGR format)
            
        Returns:
            Tuple of (processed_image, aruco_detected)
        """
        try:
            # Detect and draw ArUco markers
            corners, ids, processed_image = self.detect_markers(image)
            
            # Return processed image and detection status
            aruco_detected = ids is not None and len(ids) > 0
            return processed_image, aruco_detected
                
        except Exception as e:
            logger.error(f"Failed to process frame: {e}")
            return image, False
    
    def process_frame_to_jpeg(self, image: np.ndarray, quality: int = 85) -> Optional[bytes]:
        """
        Process OpenCV frame and return as JPEG bytes.
        
        Args:
            image: OpenCV image array (BGR format)
            quality: JPEG quality (1-100)
            
        Returns:
            JPEG bytes or None if encoding fails
        """
        try:
            # Process frame with ArUco detection
            processed_image, _ = self.process_frame(image)
            
            # Encode to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            result, encoded_img = cv2.imencode('.jpg', processed_image, encode_param)
            
            if result:
                return encoded_img.tobytes()
            else:
                logger.error("Failed to encode processed image to JPEG")
                return None
                
        except Exception as e:
            logger.error(f"Failed to process frame to JPEG: {e}")
            return None
    
    def process_frame_to_base64(self, image: np.ndarray, quality: int = 85) -> Optional[str]:
        """
        Process OpenCV frame and return as base64-encoded JPEG.
        
        Args:
            image: OpenCV image array (BGR format)
            quality: JPEG quality (1-100)
            
        Returns:
            Base64-encoded JPEG string or None if processing fails
        """
        try:
            jpeg_data = self.process_frame_to_jpeg(image, quality)
            if jpeg_data:
                return base64.b64encode(jpeg_data).decode('utf-8')
            else:
                return None
                
        except Exception as e:
            logger.error(f"Failed to process frame to base64: {e}")
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
