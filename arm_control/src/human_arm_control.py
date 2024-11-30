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
