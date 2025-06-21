#!/usr/bin/env python3
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from speed_control import speed_controller
import steering as steering
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from steering import rover_rotation , wheel_orientation_rot
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
import drive_firmware_node as drive_firmware

class drive_controller(Node):
    def __init__(self):

        super().__init__("drive_control_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        self.drive_firmware_node = drive_firmware.drive_firmware()

        #Declare field corresponding to speed control node and current state of wheels
        self.speed_controller = speed_controller()

        self.steering = steering.Steering()

        # TODO: Tune values
        self.deadzone = 0.1 
        self.turning_speed = 3200.0

        self.tank_drive_mode = False

        #Call electrical API to get current state of wheels
        self.wheel_angles = [math.pi/2]*4 #Dummy  value, update with API call
       
        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.controller_callback, 10)
        self.speedInputPublisher = self.create_publisher(Float32MultiArray, "drive_speed_input", 10)

        # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.run)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def run(self):
        #square -> acknowledged the faults, wheels will stop taking command until it is acknolewdged
        if self.gamepad_input.square_button:
            self.drive_firmware_node.clear_motor_faults(self.gamepad_input)
            self.get_logger().info("Motor faults cleared")

        #Speed given button input
        speed = self.speed_controller.updateSpeed(self.gamepad_input.x_button, self.gamepad_input.o_button)
        speed = [speed for _ in range(4)]
        msg = Float32MultiArray()
        
        #Check whether there is an input value for rover rotation
        if self.gamepad_input.r1_button or self.gamepad_input.l1_button:
            rot_inp = 0
            if (self.gamepad_input.r1_button):
                rot_inp = 1
            elif (self.gamepad_input.l1_button):
                rot_inp = -1

            #Array with the desired speed for each wheel during rover rotation
            rotation_sp = rover_rotation(self.wheel_angles, rot_inp)

            speed = [direction*self.turning_speed for direction in rotation_sp]

        # Checking for tank drive mode
        if self.gamepad_input.triangle_button:
            self.tank_drive_mode = not self.tank_drive_mode
        if self.tank_drive_mode:
            self.get_logger().info("TANK DRIVE MODE ACTIVATED - left stick controls left wheel & right stick controls right wheel")
        else:
            self.get_logger().info("TANK DRIVE MODE DEACTIVATED - left stick controls rover rotation")
            
        if self.tank_drive_mode:    
            left_speed_wheels = steering.update_left_wheel_speeds(self.gamepad_input.l_stick_y)
            right_speed_wheels = steering.update_right_wheel_speeds(self.gamepad_input.r_stick_y)
            speed = [left_speed_wheels[0], right_speed_wheels[0], left_speed_wheels[1], right_speed_wheels[1]]
            msg.Float32MultiArray()
            msg.data = [float(s) for s in speed]
            self.speedInputPublisher.publish(msg)
            return

        #Check whether gears change
        if self.gamepad_input.r2_button or self.gamepad_input.l2_button:
            self.speed_controller.shifting_gear(self.gamepad_input.r2_button, self.gamepad_input.l2_button)

        #Check whether joystick position changes
        if self.not_in_deadzone_check(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y):
            #Orientation for wheels given a joystick positinon
            self.wheel_angles = wheel_orientation_rot(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y, self.wheel_angles[0])

            # TODO: Send new orientation to wheels

        # TODO: Use API to send input values for speed, orientation and rotation.
        msg.data = [float(s) for s in speed]
        self.speedInputPublisher.publish(msg)
        
    def controller_callback(self, input: GamePadInput):
        self.gamepad_input = input


def main(args=None):
    rclpy.init(args=args)
    drive_controller_node = drive_controller()
    rclpy.spin(drive_controller_node)

if __name__ == "__main__":
    main()
        