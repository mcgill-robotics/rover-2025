import asyncio
import cv2
import numpy as np
from av import VideoFrame
from aiortc import VideoStreamTrack

class ArucoProcessorTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.pipeline = self._build_pipeline()
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

    def _build_pipeline(self):
        return (
            "udpsrc port=5000 ! application/x-rtp, encoding-name=H264, payload=96 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        for _ in range(3):
            print("Waiting for frames")
            ret, frame = self.cap.read()
            print("Received frames")
            if ret:
                break
            await asyncio.sleep(0.03)
        else:
            raise RuntimeError("Failed to receive frame from GStreamer")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame
