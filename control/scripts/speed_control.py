import scipy.stats as st
import numpy as np


class speed_controller():

    """
    Hard-coded the rover measurements and gear speeds, need to adapt it towards the real model.
    Speed_controller manages the velocity adjustments based on gear shifts and gamepad inputs.
    - Supports acceleration and deceleration through gamepad inputs
    - Implements gear shifting and a smoother transition when downshifting.
    - Keeps a history of recent wheel speeds.

    """

    def __init__(self, json_filename="max_wheel_speed_levels.json"):
        
        # (Mar.1, 2025) Max speed is 2000 rpm
        self.gears = [{
                        "gear": 1,
                        "speed": 400.0
                    }, {
                        "gear": 2,
                        "speed": 800.0
                    }, {
                        "gear": 3,
                        "speed": 1200.0
                    }, {
                        "gear": 4,
                        "speed": 1600.0
                    }, {
                        "gear": 5,
                        "speed": 2000.0
                    }]

        self.history_size = 10 # How many previous wheels we average over

        # A 1-D Gaussian array to act as a weighted average on self.wheelspeeds
        # "kernlen" must be the same as "self.history_size".
        self.basis = self.gkern(kernlen=self.history_size)

        # Window of past wheels speeds of size self.history_size
        self.wheel_speeds_history = np.zeros(self.history_size).tolist()

        self.current_gear_index = 0
        self.current_speed = 0.0
        self.max_speed = self.gears[self.current_gear_index]["speed"]

        self.acceleration_rate = 100
        self.deceleration_rate = 100  # Decay value when no acceleration input is given
        self.downshift_deceleration_rate = 200  # Slightly faster decay when downshifting

    # Function that yields a 1D gaussian
    # Source: https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    def gkern(self, kernlen=10, sigma=3):
        x      = np.linspace(-sigma, sigma, kernlen+1)
        kern1d = np.diff(st.norm.cdf(x))
        return kern1d/kern1d.sum()
    
    """
    Change absolute max speed
    - R2 shifts up a gear, L2 shifts down a gear with smooth deceleration if needed
    """
    def shifting_gear(self, gear_up: bool, gear_down: bool):
        if gear_up: # r2 switches to higher gear
            self.current_gear_index = min(self.current_gear_index + 1, len(self.gears) - 1)
        elif gear_down: # l2 switches to lower gear
            self.current_gear_index  = max(self.current_gear_index - 1, 0)
        self.max_speed = self.gears[self.current_gear_index]["speed"]
        
    # Update current speed, and add it to the self.wheelspeeds history
    def updateSpeed(self, accelerate: bool, reverse: bool):
        """
        Updates current speed based on gamepad inputs and shifting gear
        - X button accelerates, O button decelerates
        - R2 shifts up a gear, L2 shifts down a gear with smooth deceleration if needed
        """

        # Handling case where shifting to a lower gear from higher speeds, deaccelerating slightly faster
        if abs(self.current_speed) > abs(self.max_speed):
            if self.current_speed > self.max_speed:
                self.current_speed = max(self.current_speed - self.downshift_deceleration_rate, self.max_speed)
            elif self.current_speed < -self.max_speed:
                self.current_speed = min(self.current_speed + self.downshift_deceleration_rate, -self.max_speed)
        else:
            # Accelerate
            if accelerate:
                self.current_speed = min(self.current_speed + self.acceleration_rate, self.max_speed)
            # Reverse
            elif reverse:
                self.current_speed = max(self.current_speed - self.acceleration_rate, -self.max_speed)
            # if no button is pressed, then decelerate speed to zero
            else:
                if self.current_speed > 0:
                    self.current_speed = max(self.current_speed - self.deceleration_rate, 0)
                elif self.current_speed < 0:
                    self.current_speed = min(self.current_speed + self.deceleration_rate, 0)

        # Add current wheel speed to history, and pop oldest (front) value
        self.wheel_speeds_history.append(self.current_speed)
        self.wheel_speeds_history.pop(0)
        smoothed_speed = np.sum(np.array(self.wheel_speeds_history) * self.basis)

        return smoothed_speed


    def get_gear_and_max_speed(self):
        return (self.current_gear_index, self.max_speed)

