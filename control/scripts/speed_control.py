import os
import rclpy
import scipy.stats as st
from geometry_msgs.msg import Twist
from rclpy.node import Node
from msg_interface.msg import GamePadInput
import numpy as np
import json


class speed_controller():

    """
    Hard-coded the rover measurements and gear speeds, need to adapt it towards the real model.
    Speed_controller manages the velocity adjustments based on gear shifts and gamepad inputs.
    - Supports acceleration and deceleration through gamepad inputs
    - Implements gear shifting and a smoother transition when downshifting.
    - Keeps a history of recent wheel speeds.

    """

    def __init__(self, json_filename="max_wheel_speed_levels.json"):

        current_dir = os.path.dirname(os.path.realpath(__file__))
        conf_path = os.path.join(current_dir, "conf", "max_wheel_speed_levels.json")
        self.gears = self.load_gears(conf_path)

        self.history_size = 10 # In the drive_control_node it says default is 50 

        self.basis = self.gkern(kernlen=self.sample_size)  # Gaussian kernel array. "kernlen" must be the same as "self.sample_size".

        # Velocity Sample for all the wheel
        self.wheel_speeds = np.zeros(self.sample_size).tolist()

        self.current_gear_index = 0
        self.current_speed = 0.0

        self.acceleration_rate = 0.2
        self.deceleration_rate = 0.1  # Decay value when no acceleration input is given
        self.downshift_deceleration_rate = 0.15  # Slightly faster decay when downshifting

    # Function that yields a 1D gaussian basis, from drive_control_node -Aman
    # Source: https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    def gkern(self, kernlen=10, sigma=3):
        x      = np.linspace(-sigma, sigma, kernlen+1)
        kern1d = np.diff(st.norm.cdf(x))
        return kern1d/kern1d.sum()

    def load_gears(self, path):
        with open(path, "r") as f:
            data = json.load(f)
        return data["max_speeds"]
    

    def shifting_gear(self, gear_input):
        # seperate  function
        if gear_input.r2_button: # r2 switches to next gear
            self.current_gear_index = min(self.current_gear_index + 1, len(self.gears) - 1)

        if gear_input.l2_button: # l2 switches to prev gear
            new_gear_index = max(self.current_gear_index - 1, 0)
            new_max_speed = self.gears[new_gear_index]["speed"]

            # Handling case where shifting to a lower gear from higher speeds, deaccelerating slightly faster
            if self.current_speed > new_max_speed:
                self.current_gear_index = new_gear_index
                while self.current_speed > new_max_speed: # Apply gradual deceleration if downshifting from a higher speed
                    self.current_speed -= self.downshift_deceleration_rate
                    self.current_speed = max(self.current_speed, new_max_speed)
            else:
                self.current_gear_index = new_gear_index
        
    
    def updateSpeed(self, gamepad_input):

        """
        Updates current speed based on gamepad inputs and shifting gear
        - X button accelerates, O button decelerates
        - R2 shifts up a gear, L2 shifts down a gear with smooth deceleration if needed
        """
        max_speed = self.gears[self.current_gear_index]["speed"]

        if gamepad_input.x_button:
            self.current_speed += self.acceleration_rate
            if self.current_speed > max_speed:
                self.current_speed = max_speed

        elif gamepad_input.o_button:
            self.current_speed -= self.acceleration_rate
            if self.current_speed < -max_speed:
                self.current_speed = -max_speed
        
        # if no button is pressed, then apply normal decay
        else:
            if self.current_speed > 0:
                self.current_speed = max(self.current_speed - self.deceleration_rate, 0)
            elif self.current_speed < 0:
                self.current_speed = min(self.current_speed + self.deceleration_rate, 0)

        smoothed_speed = np.sum(self.current_speed * self.basis)
        self.wheel_speeds.append(smoothed_speed)
        if len(self.wheel_speeds) > 10: # remove the oldest values to keep a constant array
            self.wheel_speeds.pop(0)

        return smoothed_speed


    def get_gear_speed(self):
        return (self.current_gear_index, self.current_speed)

