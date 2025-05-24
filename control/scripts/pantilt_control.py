import rclpy
import math
from rclpy.node import Node
from msg_interface.msg import GamePadInput

import rclpy.subscription

class PanTiltController():
    """
    Class to control pan-tilt (pitch and yaw) via controller 

    Pitch: [0, pi]
    Yaw: [0, 2pi]

    Test with p4 controller
    (Make node that gets gamepad input)
    """

    def __init__(self, step_size = 0.1):
        """
        Initialize controller with default angles and step size.
        """
        self.yaw = 0.0
        self.pitch = 0.0
        self.step_size = step_size          # controlls how fast the camera moves, 0.1 rad = ~5.7


    def update_angles(self, controller):
        old_yaw = self.yaw
        old_pitch = self.pitch
        """Assuming that d_pad_x -1.0 when left pressed, 0.0 no input and +1.0 when right pressed"""
        y_delta = controller.d_pad_x * self.step_size
        self.yaw = self.adjust(self.yaw + y_delta, 0, 2*math.pi)    # to contrain according to 0 ≤ yaw ≤ 2π 
        """Assuming that d_pad_y pressedn down: +1.0 and d_pad_y pressed up: -1.0 (if so make -controller.d_pad_y)"""
        p_delta = controller.d_pad_y * self.step_size
        self.pitch = self.adjust(self.pitch + p_delta, 0, math.pi) #  to contrain according to 0 ≤ pitch ≤ π 
        return [self.yaw - old_yaw, self.pitch - old_pitch] # return the difference in angles to be used in the firmware
    

    def adjust(self, value, min_val, max_val):
        """Helper to constrain values within wanted range"""
        return max(min_val, min(value,max_val))
    
    def get_deg(self):
        return math.degrees(self.yaw), math.degrees(self.pitch)
    
    

class PanTiltSubscriber(Node):
    """Test ROS node to rest pan-tilt controller with ps4 controller"""

    def __init__(self):
        super().__init__('pan_tilt_subsriber') # activate ros node
        self.controller = PanTiltController(step_size=0.1)

        self.subscription = self.create_subscription(
            GamePadInput,
            "gampad_input",
            10
        )

        self.get_logger().info("Pan-tilt ready, use L1+D-pad for yaw, R1+D-pad for pitch")

    def listener_callback(self, msg):
        self.controller.update_angles(msg)
        yaw_deg, pitch_deg = self.controller.get_deg()
        
        self.get_logger().info(f"Yaw: {yaw_deg:6.1f} | Pitch: {pitch_deg:6.1f}")

def main():
    rclpy.init(None)
    subscriber = PanTiltSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()