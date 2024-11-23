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

def claw_control(joystick_input, cur_angle, curr_opening):
    """
    This function takes in the angle of the motor and the opening of claw,
    and returns how much the claw opens up with joystick input
    """
    #finding the change angle of opening
    #the limits of the angle must be changed in joint limit
    angle_change = speed * joystick_input * joint_max_speed[hand_index]

    if (cur_angle[hand_index] + angle_change >= joint_upper_limits[hand_index] or
    cur_angle[hand_index] + angle_change <= joint_lower_limits[hand_index]):
        return curr_opening
    
    #when the angle is at 2pi, we assume that the opening change is 1 inch
    #Also assuming the max opening to 5 inches for now
    one_turn_change = 1
    open_max = 5
    open_min = 0
    #set the new change in opening proportionally
    open_change = (angle_change*one_turn_change)/(2*np.pi)
    new_opening = curr_opening + open_change
    #ensures that new opening is not too big or small
    new_opening = max(open_min, min(new_opening, open_max))
    return new_opening


print(claw_control(-0.1, np.full(5,np.pi), 1)) #good case

print(claw_control(0.1, np.full(5,np.pi), 1)) #upper bound on angle case

print(claw_control(-0.1, np.full(5,-np.pi), 1)) #lower bound on angle case

print(claw_control(-0.1, np.full(5,0.3*np.pi), 0)) #lower bound on opening case

print(claw_control(0.1, np.full(5,0.3*np.pi), 5)) #upper bound on opening case






