import numpy as np
import math
import arm_kinematics

jointUpperLimits = [
    118.76 * np.pi / 180,
    90 * np.pi / 180,
    75 * np.pi / 180,
    75 * np.pi / 180,
    np.pi,
]  # rad
jointLowerLimits = [
    -125.97 * np.pi / 180,
    -60 * np.pi / 180,
    -70 * np.pi / 180,
    -75 * np.pi / 180,
    -np.pi,
]  # rad
jointMaxSpeed = [
    np.pi/3,
    np.pi/3,
    np.pi/3,
    np.pi/3,
    np.pi/3
] # rad per method call

speed = 1 # TO BE SET LATER
current_cycle_mode = 1 # TO BE SET LATER
joint_control_is_active = True 

def waist(joystick_input, cur_angle):
    #assumes joystick_input is normalized to be between -1.0 and 1.0
    #assumes speed to act as a scale factor (between 0.0 and 1.0)
    cur_angle[0] = cur_angle[0] + speed * normalized_joystick_input * jointMaxSpeed[0]
    if cur_angle[0] > jointUpperLimits[0]:
        cur_angle[0] = jointUpperLimits[0]
    if cur_angle[0] < jointLowerLimits[0]:
        cur_angle[0] = jointLowerLimits[0]
    return cur_angle