import math
import numpy as np

class Steering:
    def __init__(self, rWheel, base_length):
        self.rWheel=rWheel  # radius of wheel
        self.base_length=base_length # wheel base length
        
    # See link for more information: http://wiki.ros.org/diff_drive_controller
    def steering_control(self, vR, wR, maxLin=3.0, maxAng=3.0): # R = rover
        
        # Equations based on differential drive controller.
        temp=-wR*self.base_length/2
        wLeft=(vR-temp)/self.rWheel
        wRight=(vR+temp)/self.rWheel
        val = [wLeft, wRight]                     
        return val

    # rotates the wheels with joystick inputs. returns the angle of the joysticks
    def wheel_orientation_rot(self, x_input, y_input, curr_angle_rad):
        '''
        This function returns the angle at which each wheels should be oriented.
        It returns the angle of the joystick
    
        '''
        joystick_angle_rad = math.atan2(y_input, x_input)
        curr_angle_rad = joystick_angle_rad

        if curr_angle_rad < 0:
            curr_angle_rad += 2* math.pi
        return np.full(4,round(curr_angle_rad,2))
    