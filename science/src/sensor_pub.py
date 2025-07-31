import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Adjust port and baud rate as needed
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        
        # self.timer = self.create_timer(0.5, self.timer_callback)

        # Start a background thread to pull data continuously
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()


    def read_serial(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    with open('sensor_data.txt', 'a') as f:
                        f.write(line + '\n')  
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
