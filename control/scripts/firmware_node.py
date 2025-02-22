#!/usr/bin/env python3

import rclpy
import speed_control
from rclpy import Node
from msg_srv_interface.msg import GamePadInput
from steering import rover_rotation , wheel_orientation_rot

class firmware(Node):
    def __init__(self):

        #Initialize ROS node
        super.__init__("firmware_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        #Declare field corresponding to speed control node and current state of wheels
        self.speed_ctrl = speed_control()

        #TODO: Update code with API calls.
        #Call electrical API to get current state of wheels
        self.wheel_angles = []#Dummy  value, update with API call
       



        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input", self.controller_callback, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    
    def run(self):
        while rclpy.ok():
            #Check whether there is an input value for rover rotation
            if ((self.gamepad_input.r1_button != 0) or (self.gamepad_input.l1_button != 0)):
                rot_inp = 0
                if (self.gamepad_input.r1_button > 0):
                    rot_inp = 1
                elif (self.l1_button > 0):
                    rot_inp = -1


                #Array with the desired speed for each wheel during rover rotation
                rotation_sp = rover_rotation(self.wheel_angles, rot_inp)
                #Send speed to wheels

            #Check whether gears change.
            if ((self.gamepad_input.r2_button != 0) or (self.gamepad_input.l2_button != 0)):
                self.speed_ctrl.shifting_gear(self.gamepad_input)

            #Check whether joystick position changes
            if ((self.gamepad_input.l_stick_x != 0.0) or (self.gamepad_input.l_stick_y != 0.0)):
                #Orientation for wheels given a joystick positinon
                self.wheel_angles = wheel_orientation_rot(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y, self.wheel_angles[0])
                #Send new orientation to wheels

            #Speed given button input
            speed = self.speed_ctrl.updateSpeed(self.gamepad_input)

            #Use API to send input values for speed, orientation and rotation.


        
    def controller_callback(self, inp:GamePadInput):
        self.gamepad_input = inp

    def main(args=None):
        rclpy.init(args=args)
        firmware_node = firmware()
        rclpy.spin(firmware_node)

    if __name__ == "__main__":
        main()
        