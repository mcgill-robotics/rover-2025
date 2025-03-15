#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
from cv_bridge import CvBridge
import base64
import cv2

class Node_CameraFramePublishers(Node):
    def __init__(self):
        super().__init__(f'camera_frame_publisher_2')
        self.camera_idx = 2
        #rospy.init_node(f'camera_frame_publisher_{self.camera_idx}')
        #self.sub = rospy.Subscriber(f'camera_selection_{self.camera_idx}', Int16, self.selection_callback)
        self.publisher = self.create_publisher(Int16, f'camera_frame_publisher_{self.camera_idx}', 10)
        self.timer = self.create_timer(1.0/20.0, self.timer_callback)
    
        #self.webcam_publisher()
        self.cap = cv2.VideoCapture(0) # Open the webcam
        bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame_resized = cv2.resize(frame, (640, 480))
                # Convert the frame to JPEG format and compress
            _, encoded_image = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                # Convert to base64
            base64_image = base64.b64encode(encoded_image).decode('utf-8')
                # Publish as a string
            self.publisher.publish(base64_image)
    
    def selection_callback(self, msg):
        # when received a int i, close the current stream and open the new stream i
        self.cap.release()
        self.cap = cv2.VideoCapture(msg.data)
    
if __name__ == '__main__':
    rclpy.init(args=None)
    try:
        Node_CameraFramePublishers()
    except rclpy.exceptions.ROSInterruptException:
        pass