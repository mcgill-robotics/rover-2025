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

    # rotates the wheels with joystick inputs
    def wheel_orientation_rot(self, x_input, y_input, curr_angle_rad):
        joystick_angle_rad = math.atan2(y_input, x_input)
        new_angle_boundary = joystick_angle_rad
        if new_angle_boundary < 0:
            new_angle_boundary += 2* math.pi
        tolerance = 0.000001
        step_size = 0.1

        #choose if more optimal to go cw or ccw and increment by 0.1 rad till desired angle
        angle_diff = new_angle_boundary - curr_angle_rad
        if abs(angle_diff) < tolerance:
            return np.full(4,curr_angle_rad)
        
        elif 0 < angle_diff <= math.pi:
            while abs(curr_angle_rad-new_angle_boundary) > tolerance:
                if curr_angle_rad >= 2*math.pi:
                    curr_angle_rad -= 2*math.pi
                curr_angle_rad += step_size

        elif angle_diff > math.pi:
            while abs(curr_angle_rad-new_angle_boundary) > tolerance:
                if curr_angle_rad <= 0 :
                    curr_angle_rad += 2*math.pi
                curr_angle_rad -= step_size

        elif angle_diff < -math.pi:
            while abs(curr_angle_rad-new_angle_boundary) > tolerance:
                if curr_angle_rad >= 2* math.pi:
                    curr_angle_rad -= 2*math.pi
                curr_angle_rad += step_size
        
        elif -math.pi < angle_diff < 0:
            while abs(curr_angle_rad-new_angle_boundary) > tolerance:
                if curr_angle_rad <= 0:
                    curr_angle_rad += 2*math.pi
                curr_angle_rad -= step_size
        
        return np.full(4, round(curr_angle_rad,2))

