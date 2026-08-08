from rclpy import Node
import rclpy
import os
from pathlib import Path

from msg_srv_interface.srv import PanTiltSavePhoto
import shutil
from stitching import Stitcher
import cv2
import time

PANTILT_SIGNAL_FILE = "PLACEHOLDER"
PANORAMA_FOLDER = "PLACEHOLDER"
PANTILT_IMAGE_FOLDER = "PLACEHOLDER"

class PanTiltPanoramaHelper(Node):
    def __init__(self):
        super().__init__("panorama-helper")
        self.srv = self.create_service(PanTiltSavePhoto, "save-photo", self.callback)

    def callback(self, request, response):
        Path(PANTILT_SIGNAL_FILE).touch() # Signal backend to save a frame
        while os.path.exists(PANTILT_SIGNAL_FILE):
            pass

        if not os.path.exists(PANTILT_IMAGE_FOLDER):
            os.mkdri(PANTILT_IMAGE_FOLDER)

        if request.make_pano:
            stitcher = Stitcher() #TODO: check if default settings are okay
            panorama = stitcher.stitch([f"{PANTILT_IMAGE_FOLDER}/?.jpg"])
            if not os.path.isdir(PANORAMA_FOLDER):
                os.mkdir(PANORAMA_FOLDER)
            cv2.imwrite(f"{PANORAMA_FOLDER}/{time.now()}.jpg", panorama)

            shutil.rmtree(PANTILT_IMAGE_FOLDER)

        response.success = True
        return response

