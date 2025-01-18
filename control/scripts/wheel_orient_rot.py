import math
import numpy as np

def convert_joystick_to_angle(x_input, y_input):
    # x_input, y_input are the joystick inputs, range [-1,1]
    # x_input is joystick left/right
    # y_input is joystick up/down
    # x_input and y_input are in the range [-1,1]
    angle_rad = math.atan2(y_input, x_input)
    return angle_rad


# This function updates wheel orientation based on joystick input
def wheel_orientation_rot(x_input, y_input, curr_angle_rad):
    joystick_angle_rad = convert_joystick_to_angle(x_input, y_input)
    new_angle_boundary = joystick_angle_rad


    #Very fast right now, could change...
    while curr_angle_rad < new_angle_boundary:
        curr_angle_rad = 0 if curr_angle_rad >= 2*math.pi else curr_angle_rad
        curr_angle_rad += 0.01
    
    while curr_angle_rad > new_angle_boundary:
        curr_angle_rad = 0 if curr_angle_rad <= 0 else curr_angle_rad
        curr_angle_rad -= 0.01
    
    
    return np.full(4, curr_angle_rad)

#Tests
print(wheel_orientation_rot(0,1,3*(math.pi/2))) #[pi/2,pi/2,pi/2,pi/2]
print(wheel_orientation_rot(-1,0, 0)) #[pi,pi,pi,pi]
print(wheel_orientation_rot(0.5,0.5, math.pi/2)) # [pi/4, pi/4,pi/4,pi/4]

