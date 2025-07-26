#!/usr/bin/env python3
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from human_arm_control import *
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

# Enum for control schema
IK_CONTROL = 0
JOINT_CONTROL = 1

class arm_control_node(Node):
    def __init__(self):

        super().__init__("arm_control_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        self.controller = HumanArmControl() 

        # TODO: Tune values
        self.deadzone = 0.1 

        self.cur_angles = [0.0,0.0,0.0,0.0,0.0] #Dummy  value, update with API call
        self.current_schema = IK_CONTROL  # Start with Inverse Kinematics control
       
        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_arm", self.run, 10)
        self.feedbackSubscriber = self.create_subscription(Float32MultiArray, "arm_position_feedback", self.updateArmPosition, 10)

        self.position_publisher = self.create_publisher(Float32MultiArray, 'arm_position_cmd', 10)
        self.fault_publisher = self.create_publisher(Bool, "acknowledge_arm_faults", 10)

        # # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.run)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def run(self, gamepad_input: GamePadInput):
        """
        Main control loop that processes gamepad input and updates arm angles accordingly
        """
        if gamepad_input.home_button:
            msg = Bool()
            msg.data = True
            self.fault_publisher.publish(msg)
        else:
            msg = Bool()
            msg.data = False
            self.fault_publisher.publish(msg)

        new_angles = self.cur_angles # Create a copy of the current angles to modify
        #Check if there is input value for changing the speed
        if gamepad_input.square_button:
            self.controller.speed_up()
        if gamepad_input.o_button:
            self.controller.speed_down()
        
        #Check if there is input value for cycling between joints
        if gamepad_input.triangle_button:
            self.controller.cycle_up()
        if gamepad_input.x_button:
            self.controller.cycle_down()
        
        if gamepad_input.select_button:
            self.controller.set_horizontal_lock(self.cur_angles)
        
        if self.current_schema == IK_CONTROL:
            #Check if there is input value for vertical plannar motion:
            if self.not_in_deadzone_check(gamepad_input.d_pad_y, 0):
                new_angles = self.controller.vertical_motion(gamepad_input.d_pad_y, self.cur_angles)
            
            #Check if there is input value for horizontal plannar motion:
            elif self.not_in_deadzone_check(gamepad_input.d_pad_x, 0):
                new_angles = self.controller.horizontal_motion(gamepad_input.d_pad_x, self.cur_angles)
            
            #Check if there is joystick value for depth plannar motion:
            elif self.not_in_deadzone_check(gamepad_input.l_stick_y, 0):
                new_angles = self.controller.depth_motion(gamepad_input.l_stick_y, self.cur_angles)

            #Check if there is input for up and down tilt
            elif gamepad_input.r2_button:
                new_angles = self.controller.upDownTilt(1, self.cur_angles)

            elif gamepad_input.l2_button:
                new_angles = self.controller.upDownTilt(-1, self.cur_angles)
        
        elif self.current_schema == JOINT_CONTROL:
            #Check if there is joystick value for specific angle adjustment and if individual joint moment allowed
            if self.not_in_deadzone_check(gamepad_input.r_stick_y, 0):
                new_angles = self.controller.move_joint(gamepad_input.r_stick_y, self.cur_angles)
            
        
        #Check if there is input for enabling/disabling joint control
        if gamepad_input.start_button:
            if self.current_schema == IK_CONTROL:
                self.current_schema = JOINT_CONTROL
            else:
                self.current_schema = IK_CONTROL

        msg = Float32MultiArray()
        msg.data = new_angles
        self.position_publisher.publish(msg)

    def updateArmPosition(self, position: Float32MultiArray) -> None:
        """
        Callback to update the current arm angles based on feedback from the arm firmware
        """
        self.cur_angles = position.data.tolist()

def main(args=None):
    rclpy.init(args=args)
    arm_control_node = arm_control_node()
    rclpy.spin(arm_control_node)

if __name__ == "__main__":
    main()
