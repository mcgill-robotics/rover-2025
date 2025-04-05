import numpy as np
import math
from ..src import arm_kinematics

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
angle_increment = [np.pi/4, np.pi/8, np.pi/16] # TO BE SET LATER
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
            out = arm_kinematics.inverseKinematics(new_pos, cur_angles)
            if not(math.pi - math.pi/12 <= abs(cur_angles[0] - out[0]) <= math.pi + math.pi/12):
                return out
        except:
            continue

    return cur_angles

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
        # new_length = math.sqrt(ee_proj_length ** 2 + delta_horizontal ** 2)

        # #calculate new waist angle
        # new_waist_angle = math.asin(delta_horizontal/new_length) + cur_angles[0]

        #use trig to get new x and y
        new_x = cur_ee_pos[0] + delta_horizontal/ee_proj_length * cur_ee_pos[1]
        new_y = cur_ee_pos[1] + delta_horizontal/ee_proj_length * -cur_ee_pos[0]

        #call inverseKinematics
        new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
        try:
            return arm_kinematics.inverseKinematics(new_pos, cur_angles)
        except:
            continue
    return cur_angles

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
            new_angles = list(arm_kinematics.inverseKinematics(cur_pos, cur_angles))

            #if no exception raised, return
            return new_angles
        except Exception as e:
            print(e)
            pass

    return cur_angles

    #z is up (blue)
    #y is side (green)
    #x is in front (red)


def assertListEqual(list1, list2):
    if len(list1) != len(list2):
        raise AssertionError
    for i in range(len(list1)):
        assert abs(list1[i] - list2[i]) <= 0.01

if __name__ == "__main__":
    # test make wrist go up
    cur_angles = [0.13184217,0.1213004,0.35479,0.593412,0.00087]
    old_position = arm_kinematics.forwardKinematics(cur_angles)
    old_position = arm_kinematics.Mat2Pose(old_position)
    new_angles = upDownTilt(0.5, cur_angles)
    new_position = arm_kinematics.forwardKinematics(new_angles)
    new_position = arm_kinematics.Mat2Pose(new_position)
    assertListEqual(new_angles, [0.13184217, 0.11062309090561215, 0.3843967744956389, 0.1748286182178238, 0.0])
    assertListEqual(old_position[:3], new_position[:3])

    # test make wrist go down
    cur_angles_down = [0.13184217,0.1213004,0.35479,0,0.00087]
    old_position_down = arm_kinematics.forwardKinematics(cur_angles_down)
    old_position_down = arm_kinematics.Mat2Pose(old_position_down)
    new_angles_down = upDownTilt(-0.5, cur_angles_down)
    new_position_down = arm_kinematics.forwardKinematics(new_angles_down)
    new_position_down = arm_kinematics.Mat2Pose(new_position_down)
    assertListEqual(new_angles_down, [0.13184217, 0.1271976350518043, 0.3310117280892946, 0.4226906251782738, 0.0])
    assertListEqual(old_position_down[:3], new_position_down[:3])

    print("All tests passed")