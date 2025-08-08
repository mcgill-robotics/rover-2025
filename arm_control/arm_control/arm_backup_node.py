#!/usr/bin/env python3
import time, os, sys, json
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from armCANCommunicationV2 import ArmNodeID, CANStation, ArmESCInterface
import rclpy
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from human_arm_control import *
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

waist_angle_threshold    = [-90, 90]
shoulder_angle_threshold = [-90, 90]
elbow_angle_threshold    = [-90, 90]

#Preset positions 
PRESET1 = {"waist": 45.0, "shoulder": 5.0, "elbow": 5.0} #we can use this for stowed position
PRESET2 = {"waist": 45.0, "shoulder": 30, "elbow": 45.0} #this can be more of a centered position

frequency_hz = 50
step_time = 0.025

desired_dps = 5.0
JOYSTICK_SCALE = desired_dps / frequency_hz
waist_speed_dps = 100.0
WAIST_DELTA = waist_speed_dps / frequency_hz
DEADZONE = 0.1

station = CANStation(interface="slcan", channel="COM7", bitrate=500000)
arm = ArmESCInterface(station)

def run_mode(elbow_pos, shoulder_pos, waist_pos):
    arm.run_follower(ArmNodeID.ELBOW, elbow_pos)
    arm.run_follower(ArmNodeID.SHOULDER, shoulder_pos)
    arm.run_follower(ArmNodeID.WAIST, waist_pos)

def go_to_preset(preset):
    run_mode(preset["elbow"], preset["shoulder"], preset["waist"])
    print(f"Moved to preset: {preset}")

#default positions
elbow_pos    = 0.0
shoulder_pos = 0.0
waist_pos    = 0.0


class arm_backup_node(Node):
    def __init__(self):

        super().__init__("arm_backup_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()
        self.controller = HumanArmControl() 

        # TODO: Tune values
        self.deadzone = 0.1
        self.active = False
                
        self.init_calibration()
        time.sleep(0.5)

        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_arm", self.update_gamepad_input, 10)

        # # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        timer_period = 0.025
        self.timer = self.create_timer(timer_period, self.run)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def init_calibration(self):
        print("[Calibration] Triangle pressed - calibrating all joints...")
        try:
            arm.calibrate(ArmNodeID.WAIST)
            time.sleep(0.025)
            arm.calibrate(ArmNodeID.SHOULDER)
            time.sleep(0.025)
            arm.calibrate(ArmNodeID.ELBOW)
            time.sleep(0.025)
        except Exception as e:
            print(f"[Calibration] Error during calibration: {e}")

    def update_gamepad_input(self, gamepad_input: GamePadInput):
        self.gamepad_input = gamepad_input
    
    def run(self):
        """
        Main control loop that processes gamepad input and updates arm angles accordingly
        """
        
        #Check if there is input value for cycling between joints
        if self.gamepad_input.triangle_button:
            self.init_calibration()

        waist_delta = 0
        if self.gamepad_input.l1_button and not self.gamepad_input.r1_button:
            waist_delta = -WAIST_DELTA
        if self.gamepad_input.r1_button and not self.gamepad_input.l1_button:
            waist_delta = WAIST_DELTA

        if self.gamepad_input.l2_button:
            elbow_pos    = PRESET1["elbow"]
            shoulder_pos = PRESET1["shoulder"]
            waist_pos    = PRESET1["waist"]
            go_to_preset(PRESET1)

        if self.gamepad_input.r2_button:
            elbow_pos    = PRESET2["elbow"]
            shoulder_pos = PRESET2["shoulder"]
            waist_pos    = PRESET2["waist"]
            go_to_preset(PRESET2)
        
        if self.gamepad_input.x_button:
            self.active = not self.active

        if self.active:
            if self.not_in_deadzone_check(self.gamepad_input.r_stick_y, 0) or self.not_in_deadzone_check(self.gamepad_input.l_stick_y, 0):
                elbow_pos    = np.clip(elbow_pos    + self.gamepad_input.r_stick_y * JOYSTICK_SCALE, elbow_angle_threshold[0], elbow_angle_threshold[1])
                shoulder_pos = np.clip(shoulder_pos + self.gamepad_input.l_stick_y * JOYSTICK_SCALE, waist_angle_threshold[0], waist_angle_threshold[1])
                waist_pos    = np.clip(waist_pos    + waist_delta, shoulder_angle_threshold[0], shoulder_angle_threshold[1])
                
            run_mode(elbow_pos, shoulder_pos, waist_pos)


def main(args=None):
    rclpy.init(args=args)
    arm_backup = arm_backup_node()
    rclpy.spin(arm_backup)
    if station:
        station.close()

if __name__ == "__main__":
    main()

