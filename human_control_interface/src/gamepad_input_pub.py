#!/usr/bin/env python3
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

import rclpy
import rclpy.logging
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from sensor_msgs.msg import Joy
from gamepad import Gamepad
import time

class gamepad_input_publisher(Node):
    def __init__(self):
 
        # initialize ROS node
        super().__init__("gamepad_process_node")

        self.gamepad_init_successful = False
        attempt_to_connect = 0

        # Initialize a Gamepad object if controller is detected
        while True: 
            try:
                self.gamepad = Gamepad()
                self.gamepad_init_successful = True
                break
            except:
                print(f"Controller not found. Trying again to connect again | Attempt: {attempt_to_connect}")
                attempt_to_connect += 1
                if attempt_to_connect == 10:
                    raise Exception("Failed to connect to controller")
                time.sleep(1)

        #Declare publisher for Joystick msg
        # self.gamepad_publisher = self.create_publisher(Joy, "gamepad_input", 10)
        self.gamepad_publisher = self.create_publisher(GamePadInput, "gamepad_input", 10)

        # Control frequency of the node
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    # The run loop that updates a controller's value.
    def run(self):
        while rclpy.ok():
            try:

                self.gamepad.update()
                msg = GamePadInput()

                # Transfer Data into msg

                msg.x_button        = self.gamepad.data.b1
                msg.o_button        = self.gamepad.data.b2
                msg.triangle_button = self.gamepad.data.b3
                msg.square_button   = self.gamepad.data.b4
                msg.l1_button       = self.gamepad.data.b5
                msg.r1_button       = self.gamepad.data.b6
                msg.l2_button       = self.gamepad.data.b7
                msg.r2_button       = self.gamepad.data.b8
                msg.select_button   = self.gamepad.data.b9
                msg.start_button    = self.gamepad.data.b10
                msg.home_button     = self.gamepad.data.b11
                msg.l3_button       = self.gamepad.data.b12
                msg.r3_button       = self.gamepad.data.b13
                msg.l_stick_x       = self.gamepad.data.a1
                msg.l_stick_y       = self.gamepad.data.a2
                msg.l_stick_analog  = self.gamepad.data.a3
                msg.r_stick_x       = self.gamepad.data.a4
                msg.r_stick_y       = self.gamepad.data.a5
                msg.r_stick_analog  = self.gamepad.data.a6
                msg.d_pad_x         = float(self.gamepad.data.a7[0])
                msg.d_pad_y         = float(self.gamepad.data.a7[1])
              
                self.gamepad_publisher.publish(msg)

            except Exception as error:
                    print
                    self.get_logger().info(str(error))



def main(args=None):
    rclpy.init(args=args)
    gamepad_input_node = gamepad_input_publisher()
    rclpy.spin(gamepad_input_node)

if __name__ == "__main__":
    main()

