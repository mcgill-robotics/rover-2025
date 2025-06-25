import cv2

class ArucoDetector():
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

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
            frame_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))  # Green border
        else:
            frame_markers = frame
        return frame_markers

