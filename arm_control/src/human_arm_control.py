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
distance = 0.05
distance_increment = [distance, distance/2, distance/4] # TO BE SET LATER
angle_increment = [np.pi/4, np.pi/8, np.pi/16] # TO BE SET LATER
current_cycle_mode = 0 # 0 = waist, 1 = shoulder, 2 = elbow, 3 = wrist, 4 = hand, 5 = claw

def move_joint(joystick_input, cur_angles, speed):
    """ Moves a singular joint in proportion to the joystick input
    
    Params
    ------
        joystick_input : float 
            Joystick input normalized to be -1.0 <= joystick_input <= 1.0
        cur_angle : list<float>
            The current joint positions of the arm
        index : int
            Joint to be controlled, 0 <= index <= 4, with 0 = waist, 1 = shoulder,
            2 = elbow, 3 = wrist, 4 = hand
    
    Returns
    -------
        cur_angle : list<float>
            The new desired joint angles 
    """
    #assumes joystick_input is normalized to be between -1.0 and 1.0
    #assumes speed to act as a scale factor (between 0.0 and 1.0)
    cur_angles[current_cycle_mode] = cur_angles[current_cycle_mode] + speed * joystick_input * joint_max_speed[current_cycle_mode]

    #sets the waists angle to be at the limit if it exeeds the limit
    cur_angles[current_cycle_mode] = max(joint_lower_limits[current_cycle_mode], min(cur_angles[current_cycle_mode], joint_upper_limits[current_cycle_mode]))
    return cur_angles

def speed_up():
    """
    Increases the speed of movement by speed_increment.

    Returns
    -------
        speed : float
            New speed
    """
    #assume button check is done before calling this
    speed += speed_increment

def speed_down():
    """
    Decreases the speed of movement by speed_increment.

    Returns
    -------
        speed : float
            New speed
    """
    #assume button check is done before calling this
    speed -= speed_increment
  
def cycle_up():
    """
    Changes the single joint being affected to the next joint in the cycle.

    Returns
    -------
        current_cycle_mode : int
            New index representing the joint being controlled
    """
    #assume button check is done before calling this
    current_cycle_mode += 1
    current_cycle_mode %= 6
    return current_cycle_mode

def cycle_down():
    """
    Changes the single joint being affected to the previous joint in the cycle.

    Returns
    -------
        current_cycle_mode : int
            New index representing the joint being controlled
    """
    #assume button check is done before calling this
    current_cycle_mode -= 1
    current_cycle_mode %= 6
    return current_cycle_mode

def depth_motion(joystick_input, cur_angles):
    #get current x,y,z,X,Y,Z of arms
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

    #get current arm distance
    #arm as 2D line for x,y plane
    projection_line = [
        cur_ee_pos[0],
        cur_ee_pos[1],
    ]
    ee_proj_length = arm_kinematics.projection_length(projection_line, cur_ee_pos[:2])  #length of the arm on the x,y plane

    for i in range(len(distance_increment)):
        #calculate new arm distance
        new_length = ee_proj_length + joystick_input * distance_increment[i]

        #use similar triangles to calculate new x,y
        new_x = new_length / ee_proj_length * cur_ee_pos[0]
        new_y = new_length / ee_proj_length * cur_ee_pos[1]

        #call inverseKinematics
        new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
        try:
            angles = arm_kinematics.inverseKinematics(new_pos, cur_angles)
            return angles.tolist()
        except:
            print("Failed")
            continue
    return cur_angles

def vertical_motion(joystick_input, cur_angles):
    #get current x,y,z,X,Y,Z of arms
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

    for i in range(len(distance_increment)):
        #calculate new arm distance
        new_z = cur_ee_pos[2] + joystick_input * distance_increment[i]

        #call inverseKinematics
        new_pos = cur_ee_pos.copy()
        new_pos[2] = new_z
        try:
            angles = arm_kinematics.inverseKinematics(new_pos, cur_angles)
            return angles.tolist()
        except:
            print("Failed")
            continue

    return cur_angles

def horizontal_motion(joystick_input, cur_angles, old_horiz_pos):
    #get current x,y,z,X,Y,Z of arms
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

    ee_proj_length = math.sqrt(old_horiz_pos[0]**2+old_horiz_pos[1]**2)  #length of the arm on the x,y plane
    # print("ee_proj_length:", ee_proj_length)
    # print("old x:",old_horiz_pos[0])
    # print("old y:",old_horiz_pos[1])

    for i in range(len(distance_increment)):
        #calculate new length
        delta_horizontal = joystick_input * distance_increment[i]
        total_horiz_dst = delta_horizontal + math.sqrt(abs(cur_ee_pos[0]**2+cur_ee_pos[1]**2 - old_horiz_pos[0]**2+old_horiz_pos[1]**2))
        # print("total horiz:",total_horiz_dst)

        # new_length = math.sqrt(ee_proj_length ** 2 + delta_horizontal ** 2)

        # #calculate new waist angle
        # new_waist_angle = math.asin(delta_horizontal/new_length) + cur_angles[0]

        #use trig to get new x and y
        new_x = old_horiz_pos[0] + total_horiz_dst/ee_proj_length * old_horiz_pos[1]
        new_y = old_horiz_pos[1] + total_horiz_dst/ee_proj_length * -old_horiz_pos[0]

        #call inverseKinematics
        new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
        try:
            angles = arm_kinematics.inverseKinematics(new_pos, cur_angles)
            return angles.tolist()
        except:
            print("Failed")
            continue
    return cur_angles

def get_old_horiz_pos(cur_angles):
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

    #get current arm distance
    #arm as 2D line for x,y plane
    projection_line = [
        cur_ee_pos[0],
        cur_ee_pos[1],
    ]

    return projection_line
def upDownTilt(joystick_input, cur_angles):
    """Returns new angles in radians needed to change tilt. Tries amount, then half, then half again. 
    Assumes joystick_input is normalized between -1 and 1"""

    #get current x,y,z,X,Y,Z of arms (XYZ are euler)
    cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
    cur_pos = arm_kinematics.Mat2Pose(cur_matrix)
    copy = cur_pos.copy()

    for i in range(3):

        cur_pos[4] = copy[4] + speed*angle_increment[i]*joystick_input  # modify Y euler angle (pitch)
        try:
            #use inverse kinematics to get positions of each joint needed for change of tilt
            new_angles = arm_kinematics.inverseKinematics(cur_pos, cur_angles).tolist()

            #if no exception raised, return
            return new_angles
        except Exception as e:
            print(e)
            pass

    return cur_angles


    #z is up (blue)
    #y is side (green)
    #x is in front (red)