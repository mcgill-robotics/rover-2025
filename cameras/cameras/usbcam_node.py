import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import argparse
import subprocess
import re
import os

def find_video_device_by_name(camera_name: str):
    try:
        output = subprocess.check_output(['v4l2-ctl', '--list-devices'], text=True)
    except subprocess.CalledProcessError:
        raise RuntimeError("Failed to run v4l2-ctl. Is v4l-utils installed?")

    lines = output.strip().split("\n")
    device_map = {}
    current_name = None

    for line in lines:
        if not line.startswith('\t'):
            current_name = line.strip()
            device_map[current_name] = []
        elif current_name:
            device_map[current_name].append(line.strip())

    for name, paths in device_map.items():
        if camera_name.lower() in name.lower():
            for path in paths:
                if "/dev/video" in path:
                    return path

    # Extra fallback: list all video devices and use udevadm
    import glob
    video_devices = glob.glob("/dev/video*")
    for dev in video_devices:
        try:
            udev_output = subprocess.check_output(['udevadm', 'info', '--query=all', '--name', dev], text=True)
            if camera_name.lower() in udev_output.lower():
                return dev
        except subprocess.CalledProcessError:
            continue

    raise RuntimeError(f"No matching video device found for camera '{camera_name}'")

class CameraNode(Node):
    def __init__(self, cam_path):
        super().__init__('usbcam_node')
        self.publisher_ = self.create_publisher(CompressedImage, 'usbcam_image', 0)

        self.cap = cv2.VideoCapture(cam_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open camera at {cam_path}")
        else:
            self.get_logger().info(f"✅ Camera at {cam_path} opened successfully")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ Failed to capture frame")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.publisher_.publish(msg)

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 USB Camera Node')
    parser.add_argument('--cam-name', type=str, required=True, help='Name of the camera to look for (e.g. "Centerm Camera")')
    cli_args = parser.parse_args()

    try:
        cam_path = find_video_device_by_name(cli_args.cam_name)
    except RuntimeError as e:
        print(str(e))
        return

    rclpy.init(args=args)
    node = CameraNode(cam_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
