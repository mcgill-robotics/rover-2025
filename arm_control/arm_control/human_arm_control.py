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
angle_increment = [np.pi/4, np.pi/8, np.pi/16] # TO BE SET LATER

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


