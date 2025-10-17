import time
import pygame
from enum import Enum
# from armCANCommunicationV2 import ArmNodeID, CANStation, ArmESCInterface, MotorType
# from armCANCommunication  import SystemInterface, ESCInterface
from armCANCommunicationV2 import (
    ArmNodeID, CANStation, ArmESCInterface, MotorType,
    ActionType, ReadSpec
)


import json
# from armCANCommunicationV2 import *


#PS4ControllerMap
SQUARE_BUTTON = 2
CIRCLE_BUTTON = 1
TRIANGLE_BUTTON = 3

X_BUTTON = 0
RIGHT_JOYSTICK = 3
LEFT_JOYSTICK = 1
RIGHT_BUMPER = 10
LEFT_BUMPER = 9
DPAD_UP = 11
DPAD_DOWN = 12


# Start in brushless mode
motor_type = "brushless"


#Preset possitions 
PRESET1 = {"waist": 45.0, "shoulder": 5.0, "elbow": 5.0} #we can use this for stowed position
PRESET2 = {"waist": 45.0, "shoulder": 30, "elbow": 45.0} #this can be more of a centered position



frequency_hz = 50
step_time = 0.025

desired_dps = 10.0
JOYSTICK_SCALE = desired_dps / frequency_hz
waist_speed_dps = 100.0
WAIST_DELTA = waist_speed_dps / frequency_hz
DEADZONE = 0.1

mode = "run"
 
if mode == 'run':
    station = CANStation(interface="slcan", channel="COM7", bitrate=500000)
    arm = ArmESCInterface(station)
else:
    station = None
    arm = None

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()


# USE BELOW TO TEST BUTTON MAPPING
# while True:
#     pygame.event.pump()
#     for i in range(joystick.get_numbuttons()):
#         print(f"Button {i}: {joystick.get_button(i)}", end=" | ")
#     print("\r", end="")
#     time.sleep(0.1)


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



def write_angles_to_file(waist, shoulder, elbow, filename="arm_angles.json"):
    data = {
        "waist": waist,
        "shoulder": shoulder,
        "elbow": elbow
    }
    with open(filename, "w") as f:
        json.dump(data, f)

def test_mode(elbow_pos, shoulder_pos, waist_pos):
    write_angles_to_file(waist_pos, shoulder_pos, elbow_pos)

    print(f"[TEST MODE] Elbow Pos: {elbow_pos:.2f}, Shoulder Pos: {shoulder_pos:.2f}, Waist Pos: {waist_pos:.2f}")

def run_mode(elbow_pos, shoulder_pos, waist_pos):
    write_angles_to_file(waist_pos, shoulder_pos, elbow_pos)
    arm.run_follower(ArmNodeID.ELBOW, elbow_pos)
    time.sleep(0.025)
    arm.run_follower(ArmNodeID.SHOULDER, shoulder_pos)
    time.sleep(0.025)
    arm.run_follower(ArmNodeID.WAIST, waist_pos)
    time.sleep(0.025)

def run_mode_manual (elbow_pos, shoulder_pos, waist_pos):
    write_angles_to_file(waist_pos, shoulder_pos, elbow_pos)
    arm.run_position(ArmNodeID.ELBOW, elbow_pos)
    time.sleep(0.025)
    arm.run_position(ArmNodeID.SHOULDER, shoulder_pos)
    time.sleep(0.025)
    arm.run_position(ArmNodeID.WAIST, waist_pos)
    time.sleep(0.025)

def go_to_preset(preset, mode):
    if mode == 'test':
        test_mode(preset["elbow"], preset["shoulder"], preset["waist"])
    elif mode == 'run':
        run_mode_manual(preset["elbow"], preset["shoulder"], preset["waist"])
    print(f"Moved to preset: {preset}")

#default positions
elbow_pos = 0.0
shoulder_pos = 0.0
waist_pos = 0.0

print("Move your joysticks to control the motors.")
print("Press 'X' on the controller to start/stop updating.")

active = False     # Control whether updates are sent
x_was_pressed = False  # Track edge of X button

motor_type = "brushless"   # Default mode
active = False
x_was_pressed = False
manual_mode= False
try:
    while True:
        pygame.event.pump()

        # --- MODE TOGGLE ---
        if joystick.get_button(DPAD_DOWN):
            if motor_type != "brushless":
                print("[MODE] Brushless control selected.")
            motor_type = "brushless"
            time.sleep(0.15)

        if joystick.get_button(DPAD_UP):
            if motor_type != "brushed":
                print("[MODE] Brushed control selected.")
            motor_type = "brushed"
            time.sleep(0.15)

        # -------- MANUAL MODE --------
        if manual_mode:
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

                # val = input("Shoulder setpoint (deg): ")
                # if val.strip().lower() in ['c', 'cancel', 'exit']:
                #     print("[Manual Mode Cancelled: Returning to PAUSED]")
                #     active = False
                #     manual_mode = False
                #     continue
                # shoulder_manual = float(val)

                val = input("Elbow setpoint (deg): ")
                if val.strip().lower() in ['c', 'cancel', 'exit']:
                    print("[Manual Mode Cancelled: Returning to PAUSED]")
                    active = False
                    manual_mode = False
                    continue
                elbow_manual = float(val)
            except Exception as e:
                print("Invalid input, try again.")
                continue

            # waist_pos = waist_manual
            # shoulder_pos = shoulder_manual
            elbow_pos = elbow_manual

            if mode == 'test':
                test_mode(elbow_pos, shoulder_pos, waist_pos)
            elif mode == 'run':
                run_mode_manual(elbow_pos, shoulder_pos, waist_pos)
            print(f"Setpoints sent: Waist={waist_pos}°, Shoulder={shoulder_pos}°, Elbow={elbow_pos}°")
            active = False
            manual_mode = False
            continue


        # -------- ENTER MANUAL MODE --------
        if joystick.get_button(SQUARE_BUTTON):
            manual_mode = True
            # Wait until square released to avoid retrigger
            while joystick.get_button(SQUARE_BUTTON):
                pygame.event.pump()
                time.sleep(0.05)
            continue

        # -------- BRUSHLESS LOGIC --------
        if motor_type == "brushless":



            # Triangle button for calibration

            # Calibration with triangle button
           # --- inside your main loop, replacing the old TRIANGLE calibration block ---
            
            if joystick.get_button(TRIANGLE_BUTTON):
                # Pause live updates while calibrating
                was_active = active
                active = False

                print("[Calibration] Triangle pressed - calibrating selected joints…")
                try:
                    # Choose which joints you want to calibrate
                    # If you only want elbow: joints = [ArmNodeID.ELBOW]
                    # joints = [ArmNodeID.WAIST, ArmNodeID.SHOULDER, ArmNodeID.ELBOW]
                    joints = [ArmNodeID.ELBOW]
                    calibrate_blocking(arm, station, joints)
                except Exception as e:
                    print(f"[Calibration] Error during calibration: {e}")

                # Wait for triangle release to avoid retrigger
                while joystick.get_button(TRIANGLE_BUTTON):
                    pygame.event.pump()
                    time.sleep(0.05)

                # Optionally restore active state (or keep paused—your call)
                active = was_active
                continue


            # X button for toggle
            x_button = joystick.get_button(X_BUTTON)
            if x_button and not x_was_pressed:
                active = not active
                print("ACTIVE" if active else "PAUSED")
            x_was_pressed = x_button

            right_y = -joystick.get_axis(RIGHT_JOYSTICK)
            if abs(right_y) < DEADZONE:
                right_y = 0.0

            left_y = -joystick.get_axis(LEFT_JOYSTICK)
            if abs(left_y) < DEADZONE:
                left_y = 0.0

            right_press = joystick.get_button(RIGHT_BUMPER)
            left_press = joystick.get_button(LEFT_BUMPER)
            waist_delta = 0
            if left_press and not right_press:
                waist_delta = -WAIST_DELTA
            elif right_press and not left_press:
                waist_delta = WAIST_DELTA

            # Preset triggers
            L2_pressed = joystick.get_axis(2) > 0.5
            R2_pressed = joystick.get_axis(5) > 0.5

            if L2_pressed:
                elbow_pos = PRESET1["elbow"]
                shoulder_pos = PRESET1["shoulder"]
                waist_pos = PRESET1["waist"]
                go_to_preset(PRESET1, mode)
                while joystick.get_axis(2) > 0.5:
                    pygame.event.pump()
                    time.sleep(0.05)
                continue

            if R2_pressed:
                elbow_pos = PRESET2["elbow"]
                shoulder_pos = PRESET2["shoulder"]
                waist_pos = PRESET2["waist"]
                go_to_preset(PRESET2, mode)
                while joystick.get_axis(5) > 0.5:
                    pygame.event.pump()
                    time.sleep(0.05)
                continue

            if active:
                elbow_pos += right_y * JOYSTICK_SCALE
                shoulder_pos += left_y * JOYSTICK_SCALE
                waist_pos += waist_delta

                if mode == 'test':
                    test_mode(elbow_pos, shoulder_pos, waist_pos)
                elif mode == 'run':
                    if (manual_mode == True):
                        run_mode_manual(elbow_pos, shoulder_pos, waist_pos)
                    else:
                        run_mode(elbow_pos, shoulder_pos, waist_pos)
                    
                print(f"Joystick Right Y: {right_y:.2f}, Elbow Pos: {elbow_pos:.2f}")
                print(f"Joystick Left Y: {left_y:.2f}, Shoulder Pos: {shoulder_pos:.2f}")
                print(f"Waist Delta: {waist_delta:.2f}, Waist Pos: {waist_pos:.2f}")

        # -------- BRUSHED LOGIC --------
        elif motor_type == "brushed":
            print("[Brushed] Brushed motor control not yet implemented.")

        time.sleep(step_time)

except KeyboardInterrupt:
    print("Exiting gracefully...")

finally:
    joystick.quit()
    pygame.quit()
    if station:
        station.close()

