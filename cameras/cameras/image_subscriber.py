import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

frame_lock = threading.Lock()
latest_frame = None

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'usbcam_image',
            self.listener_callback,
            10
        )
        self.get_logger().info("📷 Subscribed to 'usbcam_image'")

    def listener_callback(self, msg):
        global latest_frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            with frame_lock:
                latest_frame = frame

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Age', 0)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        while True:
            with frame_lock:
                if latest_frame is None:
                    time.sleep(0.1)
                    continue
                ret, jpeg = cv2.imencode('.jpg', latest_frame)
                if not ret:
                    continue
                frame_data = jpeg.tobytes()

            self.wfile.write(b'--frame\r\n')
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(frame_data)))
            self.end_headers()
            self.wfile.write(frame_data)
            self.wfile.write(b'\r\n')
            time.sleep(0.05)  # ~20 FPS



def start_mjpeg_server():
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    print("📡 MJPEG stream at http://localhost:8080")
    server.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    threading.Thread(target=start_mjpeg_server, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
