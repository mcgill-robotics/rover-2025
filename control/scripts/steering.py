import math

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
    
    def rover_rotation(self, wheel_angles, joystick_input):
        """
        Returns the wheel speeds in array [TR, TL, BL, BR] needed for rover rotation left or right
        Assumes joystick_input normalized between [-1, 1] for x-axis (L/R)
        Wheel rotation speed is between [-1, 1] (backward, forward), value depends on joystick input depth
        """

        tolerance = math.pi/12 # value can be changed
        # wheel angle according to unit circle form 0 to 2pi

        # CASE 1: approximately perpendicular to rover
        # front of wheel facing right
        if abs(wheel_angles[0]) <= tolerance or abs(math.pi*2-wheel_angles[0]) <= tolerance: 
            if joystick_input < 0: #turn left
                # top wheels go backward, bottom wheel rotate forward
                return [i*abs(joystick_input) for i in [-1, -1, 1, 1]] 
            else: #turning right
                # top wheels go forward, bottom wheel rotate backward
                return [i*abs(joystick_input) for i in [1, 1, -1, -1]] 

        # front of wheel facing left
        elif abs(wheel_angles[0] - math.pi) <= tolerance:
            pass

            


        # case 2: approximately parallel to rover

        # case 3: / relative to rover

        # case 4: \ relative to rover

