import cv2
from aiortc import VideoStreamTrack
from av import VideoFrame

class ArucoVideoTrack(VideoStreamTrack):
    """Class that takes the video stream of a camera and draws any detected aruco markers"""

    def __init__(self, device_path):
        super().__init__()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        
        self.cap = cv2.VideoCapture(device_path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def get_aruco(self, frame):
        """Takes a frame (numpy array) and returns a list of corners and ids of detected aruco markers"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
        return corners, ids
    
    def get_drawn_aruco(self, frame):
        """Returns a frame with detected aruco markers drawn to display its ids, 
        returns the original frame if none found"""
        corners, ids = self.get_aruco(frame)
        if ids is not None:
            frame_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids=None, borderColor=(0, 255, 0))  # Green border
            for i in range(len(ids)): # draw ID on in bigger font
                corner = corners[i][0]
                top_left = tuple(corner[0].astype(int))
                cv2.putText(frame_markers, f'id={ids[i][0]}', top_left, 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=1.5, color=(0, 255, 0), thickness=3)
        else:
            frame_markers = frame
        return frame_markers
    
    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret: 
            raise Exception("Failed to read from camera")

        try:
            aruco_frame = self.get_drawn_aruco(frame)
        except:
            aruco_frame = frame

        video_frame = VideoFrame.from_ndarray(aruco_frame, format="bgr24")
        video_frame.pts = pts
        video_frame.time_base = time_base

        return video_frame
    
    def stop(self):
        self.cap.release()
        super().stop()

