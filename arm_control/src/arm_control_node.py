#!/usr/bin/env python3
import os
import sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from human_arm_control import *
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Enum for control schema
IK_CONTROL = 0
JOINT_CONTROL = 1

ARM_PRESET_1 = {"waist": 45.0, "shoulder": 5.0, "elbow": 5.0} #we can use this for stowed position
ARM_PRESET_2 = {"waist": 45.0, "shoulder": 30, "elbow": 45.0} #this can be more of a centered position

class arm_control_node(Node):
    def __init__(self):

        super().__init__("arm_control_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        self.controller = HumanArmControl() 

        # TODO: Tune values
        self.deadzone = 0.1 

        self.cur_angles = [0.0,0.0,0.0,0.0,0.0] #Dummy  value, update with API call
        self.current_schema = JOINT_CONTROL  # Start with Inverse Kinematics control
       
        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_arm", self.run, 10)
        self.feedbackSubscriber = self.create_subscription(Float32MultiArray, "arm_position_feedback", self.updateArmPosition, 10)

        self.position_publisher = self.create_publisher(Float32MultiArray, 'arm_position_cmd', 10)
        self.fault_publisher = self.create_publisher(Bool, "acknowledge_arm_faults", 10)

        self.calibration_client = self.create_client(Trigger, "calibration_service")

        # self.init_calibration()
        self.going_to_preset = False

        # # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        # timer_period = 0.25
        # self.timer = self.create_timer(timer_period, self.run)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def init_calibration(self):
        while not self.calibration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for calibration service")
        req = Trigger.Request()
        response = self.calibration_client.call_async(req)
        rclpy.spin_until_future_complete(self, response)
        if response.result():
            if response.result().success:
                self.get_logger().info(f'Calibration Result: {response.result().success}')
            else:
                self.get_logger().error(f'Calibration Result: {response.result().success}')
            
    def run(self, gamepad_input: GamePadInput):
        """
        Main control loop that processes gamepad input and updates arm angles accordingly
        """
        #self.controller.print_joint_menu()
        self.going_to_preset = False

        if gamepad_input.home_button:
            msg = Bool()
            msg.data = True
            self.fault_publisher.publish(msg)

            #Calibration
            self.init_calibration()
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

        if gamepad_input.l1_button: #Preset1
            new_angles = [ARM_PRESET_1["waist"], ARM_PRESET_1["shoulder"], ARM_PRESET_1["elbow"], self.cur_angles[3], self.cur_angles[4]] #Include James vars
            self.going_to_preset = True

        if gamepad_input.r1_button: #Preset2
            new_angles = [ARM_PRESET_2["waist"], ARM_PRESET_2["shoulder"], ARM_PRESET_2["elbow"], self.cur_angles[3], self.cur_angles[4]] #Include James vars
            self.going_to_preset = True
        
        if not self.going_to_preset:
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
                os.system("clear")
                print("Joint Control Activated")
                self.controller.print_joint_menu()
                #Check if there is joystick value for specific angle adjustment and if individual joint moment allowed
                if self.not_in_deadzone_check(gamepad_input.r_stick_y, 0):
                    new_angles = self.controller.move_joint(gamepad_input.r_stick_y, self.cur_angles)
            
        
            #Check if there is input for enabling/disabling joint control
            if gamepad_input.start_button:
                os.system("clear")
                if self.current_schema == IK_CONTROL:
                    self.current_schema = JOINT_CONTROL
                    #print("Joint Control Activated")
                    #self.controller.print_joint_menu()
                else:
                    self.current_schema = IK_CONTROL
                    print("IK Mode Activated")

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
    arm_control_ins = arm_control_node()
    rclpy.spin(arm_control_ins)

if __name__ == "__main__":
    main()
