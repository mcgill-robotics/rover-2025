import numpy as np
import math
import arm_kinematics

#[waist, shoulder, elbow, wrist, hand(twist)]
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

speed = 1 # TO BE SET LATER
current_cycle_mode = 1 # TO BE SET LATER
joint_control_is_active = True 

def move_elbow(current, controller_input):
    """Returns list of new positions after moving elbow"""
    #assumes controller input is normalized between -1.0 and 1.0
    current_position = current[:]
    new_position = current_position[2] + speed * controller_input
    if new_position >= jointUpperLimits[2]:
        new_position = jointUpperLimits[2]
    if new_position <= jointLowerLimits[2]:
        new_position = jointLowerLimits[2]
    current_position[2] = new_position
    return current_position