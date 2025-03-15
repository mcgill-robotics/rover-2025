import numpy as np
import math
import arm_kinematics

joint_upper_limits = [ 
    118.76 * np.pi / 180, # Waist
    90 * np.pi / 180, # Shoulder
    75 * np.pi / 180, # Elbow
    75 * np.pi / 180, # Wrist
    np.pi, # Hand
]  # rad

joint_lower_limits = [
    -125.97 * np.pi / 180, # Waist
    -60 * np.pi / 180, # Shoulder
    -70 * np.pi / 180, # Elbow
    -75 * np.pi / 180, # Wrist
    -np.pi, # Hand
]  # rad

joint_max_speed = [
    np.pi/3, # Waist
    np.pi/3, # Shoulder
    np.pi/3, # Elbow
    np.pi/3, # Wrist
    np.pi/3 # Hand
] # rad per method call

speed = 1 # TO BE SET LATER
speed_increment = 0.1 # TO BE SET LATER
distance_increment = [0.05, 0.05/2, 0.05/4] # TO BE SET LATER
current_cycle_mode = 0 # 0 = waist, 1 = shoulder, 2 = elbow, 3 = wrist, 4 = hand, 5 = claw
joint_control_is_active = True 

def upDownTilt(joystick_input, cur_angles):
    #get current x,y,z,X,Y,Z of arms
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

    #get current arm distance
    #arm as 2D line for x,y plane
    projection_line = [
        cur_ee_pos[0],
        cur_ee_pos[1],
    ]

    return target_pose 


if __name__ == "__main__":
    print("hello")