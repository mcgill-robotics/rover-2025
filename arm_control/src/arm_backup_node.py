#!/usr/bin/env python3
import time, os, sys, json
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from armCANCommunicationV2 import (
    ArmNodeID, CANStation, ArmESCInterface, MotorType,
    ActionType, ReadSpec
)
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

desired_dps = 10.0
JOYSTICK_SCALE = desired_dps / frequency_hz
waist_speed_dps = 100.0
WAIST_DELTA = waist_speed_dps / frequency_hz
DEADZONE = 0.1

station = CANStation(interface="slcan", channel="/dev/ttyACM1", bitrate=500000)
arm = ArmESCInterface(station)

def read_calibration_status_v2(station: CANStation, joint: ArmNodeID, retries=20, wait_s=0.25):
    """
    Polls READ_CALIBRATION for the given joint.
    Returns the last received float (expects 1.0 when calibrated), or 0.0 on timeout.
    """
    for _ in range(retries):
        station.send_STM_command(ActionType.READ, ReadSpec.READ_CALIBRATION, 0.0, True, joint)
        val = station.recv_msg(timeout=0.75)  # a bit longer to be safe
        if val is not None:
            return val
        time.sleep(wait_s)
    return 0.0

def calibrate_blocking(arm: ArmESCInterface, station: CANStation, joints):
    """
    Sends CALIBRATION to each joint and blocks until each reports 1.0 from READ_CALIBRATION.
    """
    print("[Calibration] Starting… pausing motion updates.")
    # Kick off calibration for each requested joint (sequential is safest)
    for j in joints:
        print(f"[Calibration] Commanding {j.name}…")
        arm.calibrate(j)
        time.sleep(0.05)

        # Wait for completion (expects ESC firmware to return 1.0 when done)
        print(f"[Calibration] Waiting for {j.name} to complete…")
        start = time.time()
        while True:
            status = read_calibration_status_v2(station, j)
            if status is not None and abs(status - 1.0) < 1e-3:
                print(f"[Calibration] {j.name} OK (status={status}).")
                break
            if time.time() - start > 10.0:  # 10s safety timeout per joint
                print(f"[Calibration] WARNING: {j.name} timed out (last status={status}).")
                break
            time.sleep(0.1)
    print("[Calibration] Done.")

def run_mode(elbow_pos, shoulder_pos, waist_pos):
    arm.run_follower(ArmNodeID.ELBOW, elbow_pos)
    time.sleep(0.025)
    arm.run_follower(ArmNodeID.SHOULDER, shoulder_pos)
    time.sleep(0.025)
    arm.run_follower(ArmNodeID.WAIST, waist_pos)
    time.sleep(0.025)

def run_mode_manual (elbow_pos, shoulder_pos, waist_pos):
    arm.run_position(ArmNodeID.ELBOW, elbow_pos)
    time.sleep(0.025)
    arm.run_position(ArmNodeID.SHOULDER, shoulder_pos)
    time.sleep(0.025)
    arm.run_position(ArmNodeID.WAIST, waist_pos)
    time.sleep(0.025)

def go_to_preset(preset):
    run_mode(preset["elbow"], preset["shoulder"], preset["waist"])
    print(f"Moved to preset: {preset}")

#default positions
# elbow_pos = 35.0
# shoulder_pos = 25.0
# waist_pos = 5.0


class arm_backup_node(Node):
    def __init__(self):

        super().__init__("arm_backup_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()
        self.controller = HumanArmControl() 

        # TODO: Tune values
        self.deadzone = 0.1
        self.active = False
        self.manual_mode= False

        self.elbow_pos = 35.0
        self.shoulder_pos = 25.0
        self.waist_pos = 5.0

                
        self.init_calibration()
        time.sleep(0.5)

        self.gamepadSubscriber = self.create_subscription(GamePadInput, "gamepad_input_arm", self.update_gamepad_input, 10)

        # # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.run)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
    
    def init_calibration(self):
        print("[Calibration] Triangle pressed - calibrating all joints...")
        try:
            # arm.calibrate(ArmNodeID.WAIST)
            # time.sleep(0.025)
            # arm.calibrate(ArmNodeID.SHOULDER)
            # time.sleep(0.025)
            # arm.calibrate(ArmNodeID.ELBOW)
            # time.sleep(0.025)

            # Choose which joints you want to calibrate
            # If you only want elbow: joints = [ArmNodeID.ELBOW]
            joints = [ArmNodeID.SHOULDER, ArmNodeID.ELBOW]
            #joints = [ArmNodeID.ELBOW]
            calibrate_blocking(arm, station, joints)
        except Exception as e:
            print(f"[Calibration] Error during calibration: {e}")

    def update_gamepad_input(self, gamepad_input: GamePadInput):
        self.gamepad_input = gamepad_input
    
    
    
    def run(self):
        """
        Main control loop that processes gamepad input and updates arm angles accordingly
        """
        if self.manual_mode:
            print("\n[MANUAL MODE] Enter setpoints for WAIST, SHOULDER, ELBOW in degrees.")
            print("Type 'c' and press Enter at any prompt to cancel and return to pause.")
            try:
                # val = input("Waist setpoint (deg): ")
                # if val.strip().lower() in ['c', 'cancel', 'exit']:
                #     print("[Manual Mode Cancelled: Returning to PAUSED]")
                #     active = False
                #     manual_mode = False
                #     continue
                # waist_manual = float(val)

                val = input("Shoulder setpoint (deg): ")
                if val.strip().lower() in ['c', 'cancel', 'exit']:
                    print("[Manual Mode Cancelled: Returning to PAUSED]")
                    active = False
                    self.manual_mode = False
                    #continue
                shoulder_manual = float(val)

                val = input("Elbow setpoint (deg): ")
                if val.strip().lower() in ['c', 'cancel', 'exit']:
                    print("[Manual Mode Cancelled: Returning to PAUSED]")
                    active = False
                    self.manual_mode = False
                    #continue
                elbow_manual = float(val)
            except Exception as e:
                print("Invalid input, try again.")
                #continue

            # waist_pos = waist_manual
            self.shoulder_pos = shoulder_manual
            self.elbow_pos = elbow_manual

            run_mode_manual(self.elbow_pos, self.shoulder_pos, self.waist_pos)
            print(f"Setpoints sent: Waist={self.waist_pos}°, Shoulder={self.shoulder_pos}°, Elbow={self.elbow_pos}°")
            active = False
            self.manual_mode = False
            #continue

        if self.gamepad_input.square_button:
            self.manual_mode = True


        #Check if there is input value for cycling between joints
        if self.gamepad_input.triangle_button:
            self.init_calibration()

        waist_delta = 0
        if self.gamepad_input.l1_button and not self.gamepad_input.r1_button:
            waist_delta = -WAIST_DELTA
        if self.gamepad_input.r1_button and not self.gamepad_input.l1_button:
            waist_delta = WAIST_DELTA

        if self.gamepad_input.l2_button:
            self.elbow_pos    = PRESET1["elbow"]
            self.shoulder_pos = PRESET1["shoulder"]
            self.waist_pos    = PRESET1["waist"]
            go_to_preset(PRESET1)

        if self.gamepad_input.r2_button:
            self.elbow_pos    = PRESET2["elbow"]
            self.shoulder_pos = PRESET2["shoulder"]
            self.waist_pos    = PRESET2["waist"]
            go_to_preset(PRESET2)
        
        if self.gamepad_input.x_button:
            self.active = True
            print("Activated: " + str(self.active))

        if self.active:
            if self.manual_mode:
                run_mode_manual(self.elbow_pos, self.shoulder_pos, self.waist_pos)
            elif self.not_in_deadzone_check(self.gamepad_input.r_stick_y, 0) or self.not_in_deadzone_check(self.gamepad_input.l_stick_y, 0):
                print("Using Joysticks to move:")
                # self.elbow_pos    = np.clip(self.elbow_pos    + self.gamepad_input.r_stick_y * JOYSTICK_SCALE, elbow_angle_threshold[0], elbow_angle_threshold[1])
                # self.shoulder_pos = np.clip(self.shoulder_pos + self.gamepad_input.l_stick_y * JOYSTICK_SCALE, waist_angle_threshold[0], waist_angle_threshold[1])
                # self.waist_pos    = np.clip(self.waist_pos    + waist_delta, shoulder_angle_threshold[0], shoulder_angle_threshold[1])

                self.elbow_pos += (- self.gamepad_input.r_stick_y) * JOYSTICK_SCALE
                self.shoulder_pos += (- self.gamepad_input.l_stick_y) * JOYSTICK_SCALE
                self.waist_pos += waist_delta
                
                run_mode(self.elbow_pos, self.shoulder_pos, self.waist_pos)


def main(args=None):
    rclpy.init(args=args)
    arm_backup = arm_backup_node()
    rclpy.spin(arm_backup)
    if station:
        station.close()

if __name__ == "__main__":
    main()

