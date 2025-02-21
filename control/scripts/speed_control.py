import os
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from msg_interface.msg import GamePadInput
import json


class speed_controller():
    def __init__(self, json_filename="max_wheel_speed_levels.json"):

        current_dir = os.path.dirname(os.path.realpath(__file__))
        conf_path = os.path.join(current_dir, "conf", "max_wheel_speed_levels.json")
        self.gears = self.load_gears(conf_path)

        self.current_gear_index = 0
        self.current_speed = 0.0

        self.acceleration_rate = 0.2
        self.deceleration_rate = 0.1  # Decay value when no acceleration input is given


    def load_gears(self, path):
        with open(path, "r") as f:
            data = json.load(f)
        return data["max_speeds"]
    
    def updateSpeed(self, msg):
        if msg.r2_button: # r2 switches to next gear
            self.current_gear_index = min(self.current_gear_index + 1, len(self.gears) - 1)

        if msg.l2_button: # l2 switches to prev gear
            self.current_gear_index = max(self.current_gear_index - 1, 0)

        max_speed = self.gears[self.current_gear_index]["speed"]

        if msg.x_button:
            self.current_speed += self.acceleration_rate
            if self.current_speed > max_speed:
                self.current_speed = max_speed

        elif msg.o_button:
            self.current_speed -= self.acceleration_rate
            if self.current_speed < -max_speed:
                self.current_speed = -max_speed
        
        # if no button is pressed, then decay
        else:
            if self.current_speed > 0:
                self.current_speed -= self.deceleration_rate
                if self.current_speed < 0:
                    self.current_speed = 0
            elif self.current_speed < 0:
                self.current_speed += self.deceleration_rate
                if self.current_speed > 0:
                    self.current_speed = 0

    def get_twist(self):
        twist = Twist()
        twist.linear.x = self.current_speed
        return twist
    
def gamepad_listener():
    rclpy.init()
    node = Node('gamepad_listener')
    controller = speed_controller()

    def gamepad_callback(msg):
        controller.updateSpeed(msg)
        twist = controller.get_twist()
        print(f"Updated Speed - Linear: {twist.linear.x:.2f}")
    
    subscription = node.create_subscription(GamePadInput, '/gamepad_input', gamepad_callback, 10)
    print("Listening for gamepad inputs...")
    rclpy.spin(node)


if __name__=="__main__":
    gamepad_listener()
    

