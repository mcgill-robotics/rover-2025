#!/usr/bin/env python3
"""
ArUco marker detection module for multi-camera streaming system.
Processes OpenCV frames and overlays detected ArUco markers.
Supports simultaneous detection of multiple ArUco dictionary types.
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List, Dict, Set
import base64
from dataclasses import dataclass
from collections import defaultdict

logger = logging.getLogger(__name__)

@dataclass
class MarkerInfo:
    """Information about a detected ArUco marker."""
    id: int
    corners: np.ndarray
    dictionary_name: str
    size: int  # e.g., 4 for 4x4 markers

class MultiDictArucoDetector:
    """ArUco detector that supports multiple dictionaries simultaneously."""
    
    def __init__(self, dictionary_names: List[str] = None):
        """
        Initialize ArUco detector with multiple dictionaries.
        
        Args:
            dictionary_names: List of dictionary names to use (default: ["DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100"])
        """
        if dictionary_names is None:
            dictionary_names = ["DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100"]
        
        self.dictionary_map = {
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
        
        # Initialize detectors for each dictionary
        self.detectors = {}
        self.dictionary_sizes = {}  # Maps dictionary name to marker size (e.g., 4 for 4x4)
        
        for name in dictionary_names:
            if name not in self.dictionary_map:
                logger.warning(f"Unknown dictionary type: {name}, skipping")
                continue
            
            dict_type = self.dictionary_map[name]
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
            detector_params = cv2.aruco.DetectorParameters()
            
            self.detectors[name] = {
                "dictionary": aruco_dict,
                "params": detector_params
            }
            
            # Extract marker size from name (e.g., "DICT_4X4_100" -> 4)
            size = int(name.split('X')[0].split('_')[-1])
            self.dictionary_sizes[name] = size
        
        # Detection settings
        self.marker_colors = {
            4: (0, 255, 0),    # 4x4 markers in green
            5: (255, 0, 0),    # 5x5 markers in blue
            6: (0, 0, 255),    # 6x6 markers in red
            7: (255, 255, 0)   # 7x7 markers in cyan
        }
        self.marker_border_thickness = 2
        self.text_color = (255, 255, 255)  # White
        self.text_thickness = 2
        self.text_font = cv2.FONT_HERSHEY_SIMPLEX
        self.text_scale = 0.7
        
        logger.info(f"Multi-dictionary ArUco detector initialized with dictionaries: {list(self.detectors.keys())}")
    
    def detect_markers(self, image: np.ndarray) -> Tuple[List[MarkerInfo], np.ndarray]:
        """
        Detect ArUco markers from all configured dictionaries in image.
        
        Args:
            image: OpenCV image array
            
        Returns:
            Tuple of (list of MarkerInfo, processed_image)
        """
        try:
            # Convert to grayscale for detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Process image
            processed_image = image.copy()
            all_markers: List[MarkerInfo] = []
            
            # Detect markers from each dictionary
            for dict_name, detector in self.detectors.items():
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, detector["dictionary"], parameters=detector["params"]
                )
                
                if ids is not None and len(ids) > 0:
                    # Create MarkerInfo objects for detected markers
                    size = self.dictionary_sizes[dict_name]
                    for i, marker_id in enumerate(ids.flatten()):
                        marker = MarkerInfo(
                            id=int(marker_id),
                            corners=corners[i],
                            dictionary_name=dict_name,
                            size=size
                        )
                        all_markers.append(marker)
                    
                    # Draw markers with dictionary-specific color
                    color = self.marker_colors[size]
                    cv2.aruco.drawDetectedMarkers(
                        processed_image, corners, ids,
                        borderColor=color
                    )
                    
                    # Add marker labels
                    for marker in all_markers[-len(ids):]:  # Process only newly added markers
                        # Get marker center
                        corner = marker.corners[0]
                        center_x = int(np.mean(corner[:, 0]))
                        center_y = int(np.mean(corner[:, 1]))
                        
                        # Create label text
                        text = f"{marker.size}x{marker.size} #{marker.id}"
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
            
            # Log detection results
            if all_markers:
                markers_by_type = defaultdict(list)
                for marker in all_markers:
                    markers_by_type[f"{marker.size}x{marker.size}"].append(marker.id)
                
                detection_summary = [f"{size}: {ids}" for size, ids in markers_by_type.items()]
                logger.debug(f"Detected markers: {', '.join(detection_summary)}")
            
            return all_markers, processed_image
            
        except Exception as e:
            logger.error(f"Failed to detect ArUco markers: {e}")
            return [], image
    
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
            markers, processed_image = self.detect_markers(image)
            
            # Return processed image and detection status
            aruco_detected = len(markers) > 0
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
def create_aruco_detector(dictionary_names: str | List[str] = None) -> MultiDictArucoDetector:
    """
    Create ArUco detector with specified dictionaries.
    
    Args:
        dictionary_names: Single dictionary name or list of dictionary names
            (default: ["DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100"])
        
    Returns:
        MultiDictArucoDetector instance
    """
    if isinstance(dictionary_names, str):
        dictionary_names = [dictionary_names]
    return MultiDictArucoDetector(dictionary_names)


if __name__ == "__main__":
    # Test the ArUco detector
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    # Create detector with all three sizes
    detector = create_aruco_detector(["DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100"])
    
    # Test with a sample image if provided
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
        image = cv2.imread(image_path)
        if image is not None:
            markers, processed_image = detector.detect_markers(image)
            
            if markers:
                # Group markers by size
                markers_by_size = defaultdict(list)
                for marker in markers:
                    markers_by_size[f"{marker.size}x{marker.size}"].append(marker.id)
                
                # Print detection results
                print("Detected markers:")
                for size, ids in markers_by_size.items():
                    print(f"  {size}: {ids}")
                
                # Display result
                cv2.imshow("Multi-Dictionary ArUco Detection", processed_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print("No ArUco markers detected")
        else:
            print(f"Failed to load image: {image_path}")
    else:
        print("Multi-dictionary ArUco detector module loaded successfully")
        print("Usage: python3 aruco_detector.py <image_path>")
