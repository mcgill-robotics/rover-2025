import math
import numpy as np
from speed_control import speed_controller

class Steering:
    def __init__(self, rWheel, base_length):
        self.rWheel=rWheel  # radius of wheel
        self.base_length=base_length # wheel base length

        self.speed_controller = speed_controller()
        
    # See link for more information: http://wiki.ros.org/diff_drive_controller
    def steering_control(self, vR, wR, maxLin=3.0, maxAng=3.0): # R = rover
        
        # Equations based on differential drive controller.
        temp=-wR*self.base_length/2
        wLeft=(vR-temp)/self.rWheel
        wRight=(vR+temp)/self.rWheel
        val = [wLeft, wRight]                     
        return val
      
# rotates the wheels with joystick inputs. returns the angle of the joysticks
def wheel_orientation_rot(x_input: float, y_input: float, curr_angle_rad: float) -> np.ndarray:
    '''
    This function returns the angle at which each wheels should be oriented.
    It returns the angle of the joystick

    '''
    joystick_angle_rad = math.atan2(y_input, x_input)
    curr_angle_rad = joystick_angle_rad

    if curr_angle_rad < 0:
        curr_angle_rad += 2* math.pi
    return np.full(4,round(curr_angle_rad,2))


def update_left_wheel_speeds(self, l_stick_y):
    if abs(l_stick_y) < self.deadzone:
        return [0, 0]
    
    base_speed = self.speed_controller.max_speed 
    left_speed = l_stick_y * base_speed
    return [left_speed, left_speed] # because same speed for both left wheels

def update_right_wheel_speeds(self, r_stick_y):
    if (r_stick_y) < self.deadzone:
        return [0, 0]
    
    base_speed = self.speed_controller.max_speed
    right_speed = r_stick_y * base_speed
    return [right_speed, right_speed]


def rover_rotation(wheel_angles: list[float], rotation_dir: float) -> list[float]:
    """
    Returns the wheel speeds in array [TR, TL, BL, BR] needed for rover rotation left or right
    Assumes rotation_dir normalized between [-1, 1] for x-axis (L/R)
    Wheel rotation speed is between [-1, 1] (backward, forward), value depends on joystick input depth
    """

    tolerance = math.pi/12 # value can be changed
    # wheel angle according to unit circle form 0 to 2pi

    # CASE 1: approximately perpendicular to rover
    # front of wheel facing right
    if abs(wheel_angles[0]) <= tolerance or abs(math.pi*2-wheel_angles[0]) <= tolerance: 
        if rotation_dir < 0: #turn left
            # top wheels go backward, bottom wheel rotate forward
            return [i*abs(rotation_dir) for i in [-1, -1, 1, 1]] 
        else: #turning right
            # top wheels go forward, bottom wheel rotate backward
            return [i*abs(rotation_dir) for i in [1, 1, -1, -1]] 

    # front of wheel facing left
    elif abs(wheel_angles[0] - math.pi) <= tolerance:
        if rotation_dir < 0: #turn left
            # top wheels go forward, bottom wheel rotate backward
            return [i*abs(rotation_dir) for i in [1, 1, -1, -1]] 
        else: #turning right
            # top wheels go backward, bottom wheel rotate forward
            return [i*abs(rotation_dir) for i in [-1, -1, 1, 1]]

        
    # CASE 2: approximately parallel to rover
    # front of wheel facing front
    elif abs(wheel_angles[0]-math.pi/2) <= tolerance:
        if rotation_dir < 0: #turn left
            # left wheels go backward, right wheels rotate forward
            return [i*abs(rotation_dir) for i in [1, -1, -1, 1]] 
        else: #turning right
            # left wheels go forward, right wheel rotate backward
            return [i*abs(rotation_dir) for i in [-1, 1, 1, -1]]
    
    # front of wheel facing backward
    elif abs(wheel_angles[0]-3*math.pi/2) <= tolerance:
        if rotation_dir < 0: #turn left
            # left wheels go forward, right wheels rotate backward
            return [i*abs(rotation_dir) for i in [-1, 1, 1, -1]] 
        else: #turning right
            # left wheels go backward, right wheel rotate forward
            return [i*abs(rotation_dir) for i in [1, -1, -1, 1]]


    # CASE 3: wheels / relative to rover
    # front of wheel facing front
    elif tolerance <= wheel_angles[0] <= math.pi/2-tolerance:
        if rotation_dir < 0: #turn left
            # only TL and BR run
            return [i*abs(rotation_dir) for i in [0, -1, 0, 1]]
        else: #turning right
            # only TL and BR run
            return [i*abs(rotation_dir) for i in [0, 1, 0, -1]]
    
    # front of wheel facing backwards
    elif math.pi+tolerance <= wheel_angles[0] <= 3*math.pi/2 - tolerance:
        if rotation_dir < 0: #turn left
            # only TL and BR run
            return [i*abs(rotation_dir) for i in [0, 1, 0, -1]]
        else: #turning right
            # only TL and BR run
            return [i*abs(rotation_dir) for i in [0, -1, 0, 1]]


    # CASE 4: wheels \ relative to rover
    # front of wheel facing forward
    elif math.pi/2 + tolerance <= wheel_angles[0] <= math.pi - tolerance:
        if rotation_dir < 0: #turn left
            # only TR and BL run
            return [i*abs(rotation_dir) for i in [1, 0, -1, 0]]
        else: #turning right
            # only TR and BL run
            return [i*abs(rotation_dir) for i in [-1, 0, 1, 0]]

    # front of wheel facing backward
    else: 
        if rotation_dir < 0: #turn left
            # only TR and BL run
            return [i*abs(rotation_dir) for i in [-1, 0, 1, 0]]
        else: #turning right
            # only TR and BL run
            return [i*abs(rotation_dir) for i in [1, 0, -1, 0]]
