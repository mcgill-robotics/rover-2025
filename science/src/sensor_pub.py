import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Adjust port and baud rate as needed
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {msg.data}')
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
