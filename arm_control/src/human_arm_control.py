import numpy as np
import math
import arm_kinematics

joint_upper_limits = [
    118.76 * np.pi / 180,
    90 * np.pi / 180,
    75 * np.pi / 180,
    75 * np.pi / 180,
    np.pi,
]  # rad

joint_lower_limits = [
    -125.97 * np.pi / 180,
    -60 * np.pi / 180,
    -70 * np.pi / 180,
    -75 * np.pi / 180,
    -np.pi,
]  # rad

joint_max_speed = [
    np.pi/3,
    np.pi/3,
    np.pi/3,
    np.pi/3,
    np.pi/3
] # rad per method call

speed = 1 # TO BE SET LATER
current_cycle_mode = 1 # TO BE SET LATER
joint_control_is_active = True 

waist_index = 0
shoulder_index = 1
elbow_index = 2
wrist_index = 3
hand_index = 4

def move_joint(joystick_input, cur_angle, index):
    #assumes joystick_input is normalized to be between -1.0 and 1.0
    #assumes speed to act as a scale factor (between 0.0 and 1.0)
    cur_angle[index] = cur_angle[index] + speed * joystick_input * joint_max_speed[index]

    #sets the waists angle to be at the limit if it exeeds the limit
    cur_angle[index] = max(joint_lower_limits[index], min(cur_angle[index], joint_upper_limits[index]))
    return cur_angle