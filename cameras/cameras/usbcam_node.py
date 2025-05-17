import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image, CompressedImage  # Import the Image message type from sensor_msgs
import argparse  # Import argparse for command-line arguments

class CameraNode(Node):
    def __init__(self, cam):
        super().__init__('usbcam_node')  # Initialize the Node
        self.publisher_ = self.create_publisher(CompressedImage, 'usbcam_image', 0)  # Slightly bigger queue
        # Open the specified webcam
        self.cap = cv2.VideoCapture(cam, 0)

        # Wait a bit for the camera to initialize
        import time
        time.sleep(1)

        # Check if it actually opened
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open camera at index {cam}")
        else:
            self.get_logger().info(f"✅ Camera at index {cam} opened successfully")
            
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def timer_callback(self):
        print("ANY MESSAGE HERE")
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ Failed to capture frame")
            return

        self.get_logger().info("✅ Frame captured")
        

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"

        msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
        # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # msg = self.bridge.cv2_to_imgmsg(gray_frame, encoding="mono8")
        self.publisher_.publish(msg)
    
def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 USB Camera Node')
    parser.add_argument('--cam', type=int, default=0, help='Index of the camera (default is 0)')
    cli_args = parser.parse_args()

    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = CameraNode(cli_args.cam)  # Create an instance of the CameraNode with the specified camera index
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.cap.release()  # Release the webcam
        cv2.destroyAllWindows()  # Close any OpenCV windows
        node.destroy_node()  # Destroy the ROS 2 node
        rclpy.shutdown()  # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed