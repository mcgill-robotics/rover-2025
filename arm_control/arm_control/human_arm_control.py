import numpy as np
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

speed = 1 # TO BE SET LATER
current_cycle_mode = 1 # TO BE SET LATER
joint_control_is_active = True 

def upDownTilt(joystick_input, button, cur_pose):
    """Converts controller inputs from L2 and R2 
       to up and down tilt at fixed position.

    Parameters
    --------
        joystic_input : int
            integer representing the controller input. 
            Assume that value is normalized between -1.0 and 1.0.
            Assume that L2 points down and R2 points up (relative to the current orientation)
        joystick_button : int
            integer that represent if the joystick input is L2 or R2.
            L2 is represented by 3.
            R2 is represented by 4.
        cur_pose : list(float)
            list of five joint angles representing the current pose

    Returns
    --------
        target_pose : list(float)
            list of five joint angles 
    """
    # Normalize joystick input from [-1, 1] to [0, 1]
    input = (joystick_input + 1) / 2
    input *= speed

    # Compute current pitch
    cur_mat = arm_kinematics.forwardKinematics(cur_pose)
    cur_pose = arm_kinematics.Mat2Pose(cur_mat)
    cur_euler = cur_pose[3:] # Extract Euler angles
    
    # EXPLORE OTHER WAYS TO MAKE MOVEMENT SMOOTHER
    if button == 3: # L2 controls (down)
        cur_euler[1] -= input
    elif button == 4: # R2 controls (up)
        cur_euler[1] += input
    else:
        print("Unrecognized button")
        return cur_pose # Return cur_pose unchanged
    
    # Clamp pitch between maximum and minimum joint limits
    cur_euler[1] = max(jointLowerLimits[0], min(cur_euler[1], jointUpperLimits[0]))

    hand_pose = cur_pose[0:3] + cur_euler
    target_pose = arm_kinematics.inverseKinematics(hand_pose, cur_pose)

    return target_pose
