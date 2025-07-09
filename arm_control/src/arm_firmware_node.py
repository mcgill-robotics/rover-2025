import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import armCANCommunication as aCAN
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
# from msg_srv_interface.msg import DriveMotorDiagnostic
# from msg_srv_interface.srv import DriveMotorStatus
from std_msgs.msg import Bool


# def run_motor_position(self, node: NodeID, position: float):
#         self.esc.run(RunSpec.POSITION, position, self.motor_type, node)
# Chnahe node to waist and the position is in degrees

#     def calibrate_motor(self, node: NodeID):
        # self.esc.run(RunSpec.CALIBRATION, 0.0, self.motor_type, node)
        # the position you calibarate it at when you run motor position it starts from that degree