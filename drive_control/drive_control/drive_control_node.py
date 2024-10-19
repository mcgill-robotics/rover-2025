import scipy.stats as st
from scipy.ndimage import gaussian_filter1d
from geometry_msgs.msg import Twist
from msg_srv_interface.msg import WheelSpeed
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from steering import Steering
import numpy as np
import rclpy
from rclpy.node import Node
import os
import sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


class drive_controller(Node):

    """
    Hard-coded the simulation rover measurement, need to adapt it towards the real model.
    Script takes care of converting twist velocities from controller into rover wheel velocities.
    Specifically, velocities are published to create an accelerating filter (local average filter for math geeks).
    """
    def __init__(self):
       
        super().__init__('drive_controller')
        self.wheel_radius      = 0.04688  # In meters.
        self.wheel_base_length = 0.28     # In meters.
        self.wheel_speed       = [0, 0]   # Array elements: Left wheel speed, right wheel speed.
        self.steering          = Steering(self.wheel_radius, self.wheel_base_length)
        self.sample_size       = 10       # Default 50. The greater the value, the more "smoothing" of the wheel speeds.
        self.basis             = self.gkern(kernlen=self.sample_size) # Gaussian kernel array. "kernlen" must be the same as "self.sample_size".

        # Velocity samples for four wheels
        self.front_left  = np.zeros(self.sample_size).tolist()
        self.back_left   = np.zeros(self.sample_size).tolist()
        self.front_right = np.zeros(self.sample_size).tolist()
        self.back_right  = np.zeros(self.sample_size).tolist()

        self.angular_velocity_publisher = self.create_publisher(   WheelSpeed, '/wheel_velocity_cmd',               1                         )          
        self.robot_twist_subscriber     = self.create_subscription(Twist,      "rover_velocity_controller/cmd_vel", self.twist_to_velocity, 10)

        # Control Frequency of the drive controller
        self.timer = self.create_timer(1/50, self.run)

    # Function to receive twist velocities and call the steering function to get the rover wheel velocities.
    def twist_to_velocity(self, robot_twist):
        vR               = robot_twist.linear.x
        wR               = robot_twist.angular.z
        self.wheel_speed = self.steering.steering_control(vR, wR)

    # Function that yields a 1D gaussian basis
    # Source: https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    def gkern(self, kernlen=10, sigma=3):
        x      = np.linspace(-sigma, sigma, kernlen+1)
        kern1d = np.diff(st.norm.cdf(x))
        return kern1d/kern1d.sum()

    def run(self):
        cmd = WheelSpeed()  # Create the wheel speed message.
        print(f"Desired speed: {self.wheel_speed}")

        # Velocity filtering:

        # Append the most recent steering values.
        self.front_left .append(self.wheel_speed[0])
        self.back_left  .append(self.wheel_speed[0])
        self.front_right.append(self.wheel_speed[1])
        self.back_right .append(self.wheel_speed[1])

        # Remove the oldest values to keep the array a constant length.
        self.front_left .pop(0)
        self.back_left  .pop(0)
        self.front_right.pop(0)
        self.back_right .pop(0)

        front_left  = np.asarray(self.front_left)
        back_left   = np.asarray(self.back_left)
        front_right = np.asarray(self.front_right)
        back_right  = np.asarray(self.back_right)

        # Local average filter calculation.
        correct_lf = np.sum(front_left  * self.basis)
        correct_lb = np.sum(back_left   * self.basis)
        correct_rf = np.sum(front_right * self.basis)
        correct_rb = np.sum(back_right  * self.basis)

        # Populate the message with the averaged values.
        # cmd.left[0], cmd.left[1] = correct_lf, correct_lb
        # cmd.right[0], cmd.right[1] = correct_rf, correct_rb
        cmd.left[0]  = min(max(correct_lf, -50), 50)
        cmd.left[1]  = min(max(correct_lb, -50), 50)
        cmd.right[0] = min(max(correct_rf, -50), 50)
        cmd.right[1] = min(max(correct_rb, -50), 50)

        print(cmd)

        # Send the angular speeds.
        self.angular_velocity_publisher.publish(cmd)

        print(cmd)


# ROS runtime main entry point
def main():
    rclpy.init()
    drive_control_node = drive_controller()
    rclpy.spin(drive_control_node)
    drive_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()