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

def move_joint(joystick_input, cur_angle, index):
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
    cur_angle[index] = cur_angle[index] + speed * joystick_input * joint_max_speed[index]

    #sets the waists angle to be at the limit if it exeeds the limit
    cur_angle[index] = max(joint_lower_limits[index], min(cur_angle[index], joint_upper_limits[index]))
    return cur_angle

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
    return speed

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
    return speed
  
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
            return arm_kinematics.inverseKinematics(new_pos, cur_angles)
        except:
            continue
    return cur_ee_pos

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
            return arm_kinematics.inverseKinematics(new_pos, cur_angles)
        except:
            continue

    return cur_ee_pos

def horizontal_motion(joystick_input, cur_angles):
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
        #calculate new length
        delta_horizontal = joystick_input * distance_increment[i]
        new_length = math.sqrt(ee_proj_length ** 2 + delta_horizontal ** 2)

        #calculate new waist angle
        new_waist_angle = math.asin(delta_horizontal/new_length) + cur_angles[0]

        #use trig to get new x and y
        new_x = new_length * math.cos(new_waist_angle)
        new_y = new_length * math.sin(new_waist_angle)

        #call inverseKinematics
        new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
        try:
            return arm_kinematics.inverseKinematics(new_pos, cur_angles)
        except:
            continue
    return cur_ee_pos

# print(depth_motion(0,[0,0,0,0,0]))
# print(depth_motion(-1,[0,0,0,0,0]))
# print(depth_motion(1,[0,0,0,0,0]))



# arm_kinematics.inverseKinematics(new_pos, cur_ee_pos)




# cur_matrix = arm_kinematics.forwardKinematics([0,0,0,0,0])
# pos = arm_kinematics.Mat2Pose(cur_matrix)
# print(pos)
# #print(arm_kinematics.inverseKinematics(pos, [0.1 * np.pi,0.1 * np.pi,-0.1 * np.pi,-0.1 * np.pi,0.01 * np.pi]))
# print(
#     arm_kinematics.Mat2Pose(
#         arm_kinematics.forwardKinematics(
#             depth_motion(-1, [0,0,0,0,0])
#         )
#     )
#     )
# print(arm_kinematics.Mat2Pose(
#         arm_kinematics.forwardKinematics(
#             vertical_motion(-1, [0,0,0,0,0])
#         )
#     )
#     )
# print(arm_kinematics.Mat2Pose(
#         arm_kinematics.forwardKinematics(
#             horizontal_motion(-1, [0,0,0,0,0])
#         )
#     )
#     )