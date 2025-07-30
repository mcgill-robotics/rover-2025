#!/usr/bin/env python3
"""
Test script for the integrated GStreamer-based camera system.
Tests the GStreamer reader, ArUco detector, and backend integration.
"""

import cv2
import numpy as np
import logging
import time
import asyncio
import threading
from typing import Optional

from gstreamer_reader import GStreamerCameraReader, test_gstreamer_reader
from aruco_detector import create_aruco_detector
from central_backend import MultiCameraBackend

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_aruco_detector():
    """Test ArUco detector with a sample image."""
    logger.info("Testing ArUco detector...")
    
    try:
        # Create detector
        detector = create_aruco_detector("DICT_4X4_100")
        
        # Create a test image with ArUco markers
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255  # White background
        
        # Generate a simple ArUco marker (this is just for testing)
        # In practice, you'd use cv2.aruco.generateImageMarker()
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, 0, 200)
        
        # Place marker in test image
        test_image[100:300, 200:400] = cv2.cvtColor(marker_image, cv2.COLOR_GRAY2BGR)
        
        # Test detection
        corners, ids, processed_image = detector.detect_markers(test_image)
        
        if ids is not None and len(ids) > 0:
            logger.info(f"✅ ArUco detector test passed - detected {len(ids)} markers: {ids.flatten().tolist()}")
            
            # Test frame processing
            processed_frame, aruco_detected = detector.process_frame(test_image)
            if aruco_detected:
                logger.info("✅ Frame processing test passed")
            else:
                logger.warning("⚠️ Frame processing test failed - no markers detected")
                
            # Test base64 encoding
            b64_result = detector.process_frame_to_base64(test_image)
            if b64_result:
                logger.info("✅ Base64 encoding test passed")
            else:
                logger.warning("⚠️ Base64 encoding test failed")
                
        else:
            logger.warning("⚠️ ArUco detector test failed - no markers detected")
            
    except Exception as e:
        logger.error(f"❌ ArUco detector test failed: {e}")

def test_gstreamer_reader_basic(port: int = 5004, duration: int = 5):
    """Test GStreamer reader without actual stream."""
    logger.info(f"Testing GStreamer reader on port {port}...")
    
    try:
        # This will likely fail without an actual stream, but we can test the setup
        reader = GStreamerCameraReader(port, f"test_camera_{port}")
        
        if reader.is_active():
            logger.info(f"✅ GStreamer reader test passed - pipeline opened on port {port}")
            logger.info(f"Resolution: {reader.get_resolution()}")
            logger.info(f"Frame rate: {reader.get_frame_rate()}")
            
            # Try to read a few frames
            frame_count = 0
            start_time = time.time()
            
            while time.time() - start_time < duration and frame_count < 10:
                frame = reader.read_frame()
                if frame is not None:
                    frame_count += 1
                    logger.info(f"Received frame {frame_count}: {frame.shape}")
                else:
                    time.sleep(0.1)
            
            if frame_count > 0:
                logger.info(f"✅ Frame reading test passed - received {frame_count} frames")
            else:
                logger.warning("⚠️ No frames received (expected if no stream is running)")
                
        else:
            logger.warning(f"⚠️ GStreamer reader test failed - could not open pipeline on port {port}")
            logger.info("This is expected if no RTP stream is running on this port")
            
        reader.release()
        
    except Exception as e:
        logger.warning(f"⚠️ GStreamer reader test failed: {e}")
        logger.info("This is expected if no RTP stream is running")

def test_backend_startup():
    """Test backend startup and basic functionality."""
    logger.info("Testing backend startup...")
    
    try:
        # Create backend instance
        backend = MultiCameraBackend(http_port=8002, enable_aruco=True)  # Use different port for testing
        
        # Test camera addition
        logger.info("Testing camera addition...")
        
        # This should work even without actual streams
        backend.add_default_cameras()
        
        if len(backend.cameras) > 0:
            logger.info(f"✅ Backend test passed - added {len(backend.cameras)} cameras")
            for cam_id, cam_info in backend.cameras.items():
                logger.info(f"  Camera: {cam_id} on port {cam_info.port}")
        else:
            logger.warning("⚠️ Backend test failed - no cameras added")
        
        # Test cleanup
        backend.camera_readers.release_all()
        logger.info("✅ Backend cleanup test passed")
        
    except Exception as e:
        logger.error(f"❌ Backend test failed: {e}")

def test_integration_workflow():
    """Test the complete integration workflow."""
    logger.info("Testing complete integration workflow...")
    
    try:
        # 1. Test ArUco detector
        logger.info("Step 1: Testing ArUco detector...")
        detector = create_aruco_detector("DICT_4X4_100")
        
        # Create test frame
        test_frame = np.ones((480, 640, 3), dtype=np.uint8) * 128  # Gray background
        
        # Process frame
        processed_frame, aruco_detected = detector.process_frame(test_frame)
        logger.info(f"ArUco processing: detected={aruco_detected}")
        
        # 2. Test frame encoding
        logger.info("Step 2: Testing frame encoding...")
        _, encoded = cv2.imencode(".jpg", processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if encoded is not None and len(encoded) > 0:
            logger.info(f"✅ Frame encoding successful: {len(encoded)} bytes")
        else:
            logger.error("❌ Frame encoding failed")
            return
        
        # 3. Test base64 encoding
        logger.info("Step 3: Testing base64 encoding...")
        import base64
        b64_data = base64.b64encode(encoded).decode("utf-8")
        if b64_data and len(b64_data) > 0:
            logger.info(f"✅ Base64 encoding successful: {len(b64_data)} characters")
        else:
            logger.error("❌ Base64 encoding failed")
            return
        
        logger.info("✅ Integration workflow test passed")
        
    except Exception as e:
        logger.error(f"❌ Integration workflow test failed: {e}")

def run_all_tests():
    """Run all integration tests."""
    logger.info("=" * 60)
    logger.info("STARTING INTEGRATION TESTS")
    logger.info("=" * 60)
    
    # Test 1: ArUco detector
    logger.info("\n" + "=" * 40)
    logger.info("TEST 1: ArUco Detector")
    logger.info("=" * 40)
    test_aruco_detector()
    
    # Test 2: GStreamer reader (basic)
    logger.info("\n" + "=" * 40)
    logger.info("TEST 2: GStreamer Reader")
    logger.info("=" * 40)
    test_gstreamer_reader_basic(5004, 2)  # Short duration for testing
    
    # Test 3: Backend startup
    logger.info("\n" + "=" * 40)
    logger.info("TEST 3: Backend Startup")
    logger.info("=" * 40)
    test_backend_startup()
    
    # Test 4: Integration workflow
    logger.info("\n" + "=" * 40)
    logger.info("TEST 4: Integration Workflow")
    logger.info("=" * 40)
    test_integration_workflow()
    
    logger.info("\n" + "=" * 60)
    logger.info("INTEGRATION TESTS COMPLETED")
    logger.info("=" * 60)

def main():
    """Main test function."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Test the integrated camera system")
    parser.add_argument("--test", choices=["all", "aruco", "gstreamer", "backend", "workflow"], 
                       default="all", help="Which test to run")
    parser.add_argument("--port", type=int, default=5004, help="Port for GStreamer test")
    parser.add_argument("--duration", type=int, default=5, help="Duration for GStreamer test")
    
    args = parser.parse_args()
    
    if args.test == "all":
        run_all_tests()
    elif args.test == "aruco":
        test_aruco_detector()
    elif args.test == "gstreamer":
        test_gstreamer_reader_basic(args.port, args.duration)
    elif args.test == "backend":
        test_backend_startup()
    elif args.test == "workflow":
        test_integration_workflow()

if __name__ == "__main__":
    main()
