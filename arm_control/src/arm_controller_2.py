import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import arm_kinematics
import numpy as np
import rclpy
from rclpy.node import Node
import math
from msg_interface.msg import ArmControllerInput
from std_msgs.msg import Float32MultiArray
from arm_kinematics import jointLowerLimits, jointUpperLimits
from std_msgs.msg import String

class Node_ArmControl(Node):
    def __init__(self):
        self.num_joints = 5

        self.current_pos = [0.0] * self.num_joints
        self.desired_pos = [0.0] * self.num_joints

        self.cycle_pos = 0
        self.schema_mode = 0 # 0 for IK, 1 for joint control

        super().__init__("arm_control")
        