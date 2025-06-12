#!/usr/bin/env python3
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from std_msgs.msg import Float32MultiArray
#from steering import rover_rotation , wheel_orientation_rot
from arm_control.src.human_arm_control import *
import math
import numpy as np

# ### TEMP for Drive Test ###
# import socket
# import pygame
# import time


# JETSON_IP = "192.168.0.101"  # IP of the motor computer
# UDP_PORT = 5005           # Port to send data
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

# def send_UDP_message(msg):
#     sock.sendto(msg.encode(), (JETSON_IP, UDP_PORT))
# ### TEMP for Drive Test ###

class arm_contol_node(Node):
    def __init__(self):

        super().__init__("arm_control_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        # TODO: Tune values
        self.deadzone = 0.1 
        self.turning_speed = 500.0
        self.speed = 1

        #self.cur_angles = [0,0,0,0,0] #Dummy  value, update with API call

        self.joint_control_active = False
       
        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_arm", self.controller_callback, 10)

        self.brushed_angles = [0.0, 0.0, 0.0]
        self.brushless_angles = [0.0, 0.0, 0.0]
        self.old_horiz_position = [0.0,0.0]
        self.brushless_publisher_ = self.create_publisher(Float32MultiArray, 'armBrushlessCmd', 10)
        self.brushed_publisher_ = self.create_publisher(Float32MultiArray, 'armBrushedCmd', 10)
        self.armBrushedSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushedFb",
            self.updateArmBrushedSim,
            10
        )

        self.armBrushlessSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushlessFb",
            self.updateArmBrushlessSim,
            10
        )
        # timer_period = 2  # seconds
        # self.timer = self.create_timer(timer_period, self.run)

        # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.run)

    def updateArmBrushedSim(self, cmds):
        self.brushed_angles = cmds.data
        #print("Brushed")
    
    def updateArmBrushlessSim(self, cmds):
        self.brushless_angles = cmds.data
        #print("Brushless")

    def not_in_deadzone_check(self, x_axis, y_axis):
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def run(self):
        brushless_msg = Float32MultiArray()
        brushed_msg = Float32MultiArray()
        cur_angles = list(self.brushless_angles)
        cur_angles.append(self.brushed_angles[2])
        cur_angles.append(self.brushed_angles[1])
        cur_angles_rad = [x*math.pi/180 for x in cur_angles]


        #Check if there is input value for changing the speed
        if self.gamepad_input.square_button:
            self.speed = speed_up(self.speed)
        if self.gamepad_input.o_button:
            self.speed = speed_down(self.speed)
        
        #Check if there is input value for cycling between joints
        if self.gamepad_input.triangle_button:
            cycle_up()
        if self.gamepad_input.x_button:
            cycle_down()
        
        #Check if there is input value for vertical plannar motion:
        if self.not_in_deadzone_check(self.gamepad_input.d_pad_y, 0):
            print("vert")
            new_angles = vertical_motion(self.gamepad_input.d_pad_y, cur_angles_rad)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            #print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)

            #self.cur_angles = vertical_motion(self.gamepad_input.d_pad_y, self.cur_angles)
            #TODO: Send angles to arm
        
        #Check if there is input value for horizontal plannar motion:
        if self.not_in_deadzone_check(self.gamepad_input.d_pad_x, 0):
            print("horiz")
            new_angles = horizontal_motion(self.gamepad_input.d_pad_x, cur_angles_rad, self.old_horiz_position)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            #print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)

            #self.cur_angles = horizontal_motion(self.gamepad_input.d_pad_x, self.cur_angles)
            #TODO: Send angles to arm
        
        #Check if there is joystick value for depth plannar motion:
        if self.not_in_deadzone_check(self.gamepad_input.l_stick_y, 0):
            print("depth")
            print(cur_angles_rad)

            new_angles = depth_motion(self.gamepad_input.l_stick_y, cur_angles_rad)
            print(new_angles)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            #print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)

            #self.cur_angles = depth_motion(self.gamepad_input.l_stick_y, self.cur_angles)
            #TODO: Send angles to arm

        #Check if there is joystick value for specific angle adjustment and if individual joint moment allowed
        if self.not_in_deadzone_check(self.gamepad_input.r_stick_y, 0) and self.joint_control_active:
            new_angles = move_joint(self.gamepad_input.r_stick_y, cur_angles, self.speed)

            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )
            #print(brushed_msg.data)
            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)
            #TODO: Send angles to arm

        #Check if there is input for up and down tilt
        if self.gamepad_input.r2_button or self.gamepad_input.l2_button:
            new_angles = upDownTilt(self.gamepad_input.l2_button-self.gamepad_input.r2_button, cur_angles_rad)
            brushless_msg.data = (
                new_angles[2] * 180 / math.pi,
                new_angles[1] * 180 / math.pi,
                new_angles[0] * 180 / math.pi
            )
            #print(brushless_msg.data)
            brushed_msg.data = (
                0.0,
                new_angles[4] * 180 / math.pi,
                new_angles[3] * 180 / math.pi
            )

            self.brushless_publisher_.publish(brushless_msg)
            self.brushed_publisher_.publish(brushed_msg)            
        
        #Check if there is input for enabling/disabling joint control
        if self.gamepad_input.start_button:
            self.joint_control_active = not self.joint_control_active

        if self.gamepad_input.select_button:
            self.old_horiz_position = get_old_horiz_pos(cur_angles_rad)
            print(self.old_horiz_position)

        #command = ":".join(map(str, speed))
        #send_UDP_message(command)
        
    def controller_callback(self, input: GamePadInput):
        self.gamepad_input = input
        self.run()

def main(args=None):
    rclpy.init(args=args)
    firmware_node = arm_contol_node()
    rclpy.spin(firmware_node)

if __name__ == "__main__":
    main()
        