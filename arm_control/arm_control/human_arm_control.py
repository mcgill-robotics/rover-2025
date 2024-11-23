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

ADD_CONSTANTS = np.ones(5)

WAIST_INDEX = 0
SHOULDER_INDEX = 1
ELBOW_INDEX = 2
WRIST_INDEX = 3
HAND_INDEX = 4

speed = 1 # TO BE SET LATER
current_cycle_mode = 1 # TO BE SET LATER
joint_control_is_active = True 


def shoulder_control(curr_angles,joystick_input_y):
    # joystick input is a value -1 to 1
    curr_shoulder_angle = curr_angles[SHOULDER_INDEX]
    new_shoulder_angle = curr_shoulder_angle + (speed*joystick_input_y*ADD_CONSTANTS[SHOULDER_INDEX])


    if (jointUpperLimits[SHOULDER_INDEX] > new_shoulder_angle and
    jointLowerLimits[SHOULDER_INDEX] < new_shoulder_angle):
        return new_shoulder_angle

    return curr_shoulder_angle


#testing shoulder
for i in range(-100, 100):
    print("upper_bound",shoulder_control(np.full(5,jointUpperLimits[SHOULDER_INDEX]), i/100))
    print("lower_bound",shoulder_control(np.full(5,jointLowerLimits[SHOULDER_INDEX]), -i/100))
    print("other1",shoulder_control(np.full(5,0.2*np.pi), i/100))
    print("other2",shoulder_control(np.full(5,-0.1*np.pi), -i/100))
















