import cv2
import re
import time
from aiortc import VideoStreamTrack
from av import VideoFrame

class ArucoVideoTrack(VideoStreamTrack):
    """
    A VideoStreamTrack that uses OpenCV to capture video frames from a camera
    and overlays ArUco marker detections onto the frames.
    """

    def __init__(self, device_path, use_gstreamer=False, max_retries=3, retry_delay=1.0):
        super().__init__()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.cap = None
        self.device_index = self._parse_device_path(device_path)
        self._open_camera(use_gstreamer, max_retries, retry_delay)

    def _parse_device_path(self, path):
        if isinstance(path, str) and path.startswith("/dev/video"):
            return int(re.sub(r"\D", "", path))
        try:
            return int(path)
        except ValueError:
            raise ValueError(f"Invalid device path or index: {path}")

    def _open_camera(self, use_gstreamer, max_retries, retry_delay):
        for attempt in range(max_retries):
            try:
                if use_gstreamer:
                    gst_pipeline = (
                        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
                        "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
                    )
                    self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                else:
                    self.cap = cv2.VideoCapture(self.device_index)

                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

                if self.cap.isOpened():
                    print(f"[ArucoVideoTrack] Camera opened on attempt {attempt+1}")
                    return
                else:
                    print(f"[ArucoVideoTrack] Camera not opened on attempt {attempt+1}")
            except Exception as e:
                print(f"[ArucoVideoTrack] Error opening camera on attempt {attempt+1}: {e}")

            time.sleep(retry_delay)

        raise RuntimeError(f"[ArucoVideoTrack] Could not open camera after {max_retries} attempts.")

    def _detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        return corners, ids

    def _draw_aruco(self, frame, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids=ids, borderColor=(0, 255, 0))
            for i, corner in enumerate(corners):
                top_left = tuple(corner[0][0].astype(int))
                cv2.putText(
                    frame,
                    f'id={ids[i][0]}',
                    top_left,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1.2,
                    color=(0, 255, 0),
                    thickness=2
                )
        return frame

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        for _ in range(3):  # retry up to 3 times before failing
            ret, frame = self.cap.read()
            if ret and frame is not None:
                break
            print("[ArucoVideoTrack] Frame read failed, retrying...")
            await asyncio.sleep(0.05)
        else:
            raise RuntimeError("Failed to read frame from camera after retries")

        try:
            corners, ids = self._detect_aruco(frame)
            frame = self._draw_aruco(frame, corners, ids)
        except Exception as e:
            print(f"[ArucoVideoTrack] ArUco detection failed: {e}")

        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        print("[ArucoVideoTrack] Stopping video track...")
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().stop()
