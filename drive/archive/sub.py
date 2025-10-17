import rclpy
from rclpy.node import Node
import time
from msg_srv_interface.msg import WheelSpeed


class wheelspeed_subscriber(Node):
    def __init__(self):
        super().__init__('test_sub')
        #self.subscriber = self.create_subscription(WheelSpeed, '/wheel_velocity_cmd', printValues)
        self.subscriber = self.create_subscription(WheelSpeed, '/feedback_velocity', self.printValues, 10)
    
    def printValues(self, msg):
        time.sleep(0.1)
        print(msg)

def main():
    rclpy.init()
    test_sub = wheelspeed_subscriber()
    rclpy.spin(test_sub)
    test_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    