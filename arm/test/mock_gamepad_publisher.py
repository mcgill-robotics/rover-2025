# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gamepad_publisher')
        self.publisher_ = self.create_publisher(GamePadInput, 'gamepad_input_arm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.gamepad_callback)

    def gamepad_callback(self):
        gamepad_input = GamePadInput()
        type_of_input = input("What button/joystick?")
        if type_of_input == "s":                                #speed up
            gamepad_input.square_button = 1
        elif type_of_input == "x":                              #cycle down
            gamepad_input.x_button = 1
        elif type_of_input == "t":                              #cycle up
            gamepad_input.triangle_button = 1
        elif type_of_input == "o":                              #speed down
            gamepad_input.o_button = 1
        elif type_of_input == "select":                         #set plane for horiz motion
            gamepad_input.select_button = 1
        elif type_of_input == "start":                          #enable disable joint ctrl
            gamepad_input.start_button = 1
        elif type_of_input == "dy":                             #vert motion
            gamepad_input.d_pad_y = float(input("-1 or 1"))
        elif type_of_input == "dx":                             #horiz motion
            gamepad_input.d_pad_x = float(input("-1 or 1"))
        elif type_of_input == "ly":                             #depth motion
            gamepad_input.l_stick_y = float(input("-1 to 1"))
        elif type_of_input == "ry":                             #joint adjustment
            gamepad_input.r_stick_y = float(input("-1 to 1"))
        elif type_of_input == "l2":                             #up tilt
            gamepad_input.l2_button = int(input("1 or 0"))
        elif type_of_input == "r2":                             #down tilt
            gamepad_input.r2_button = int(input("1 or 0"))
        self.publisher_.publish(gamepad_input)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()