#!/usr/bin/env python3
import math
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from rclpy.node import Node
#from msg_srv_interface.msg import GamePadInput
from std_msgs.msg import Float32MultiArray
#from steering import rover_rotation , wheel_orientation_rot



# ### TEMP for Drive Test ###
# import socket
# import pygame
# import time


# JETSON_IP = "192.168.0.101"  # IP of the motor computer
# UDP_PORT = 5005           # Port to send data
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket

# def send_UDP_message(msg):
#     sock.sendto(msg.encode(), (JETSON_IP, UDP_PORT))
# ### TEMP for Drive Test ###

class sim_bridge_node(Node):
    def __init__(self):

        super().__init__("sim_bridge_node")

    
        self.brushed_angles = [0.0, 0.0, 0.0]
        self.brushless_angles = [0.0, 0.0, 0.0]

        self.feedback_publisher = self.create_publisher(Float32MultiArray, "arm_position_feedback", 10) #data should be in radians
        self.brushless_publisher = self.create_publisher(Float32MultiArray, 'armBrushlessCmd', 10)
        self.brushed_publisher = self.create_publisher(Float32MultiArray, 'armBrushedCmd', 10)

        self.armBrushedSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushedFb",
            self.updateArmBrushedSim,
            10
        )

        self.armBrushlessSubscriber = self.create_subscription(
            Float32MultiArray,
            "armBrushlessFb",
            self.updateArmBrushlessSim,
            10
        )

        self.positon_subscriber = self.create_subscription(
            Float32MultiArray,
            "arm_position_cmd",
            self.updateArmPosition,
            10
        )

    def updateArmBrushedSim(self, cmds):
        self.brushed_angles = [x * math.pi / 180 for x in cmds.data]    #sim gives degrees, convert deg to rad
        msg = Float32MultiArray()
        msg.data = [self.brushless_angles[2], 
                    self.brushless_angles[1], 
                    self.brushless_angles[0],
                    self.brushed_angles[2],
                    self.brushed_angles[1]]
        self.feedback_publisher.publish(msg)
    
    def updateArmBrushlessSim(self, cmds):
        self.brushless_angles = [x * math.pi / 180 for x in cmds.data]
        msg = Float32MultiArray()
        msg.data = [self.brushless_angles[2], 
                    self.brushless_angles[1], 
                    self.brushless_angles[0],
                    self.brushed_angles[2],
                    self.brushed_angles[1]]
        self.feedback_publisher.publish(msg)
        

    def updateArmPosition(self, cmds: Float32MultiArray):
        data = [x * 180 / math.pi for x in cmds.data]    #sim takes degrees, convert rads to degrees
        brushless_msg = Float32MultiArray()
        brushless_msg.data = [data[2], data[1], data[0]]
        brushed_msg = Float32MultiArray()
        brushed_msg.data = [0.0, data[4], data[3]]
        print("pub")

        self.brushless_publisher.publish(brushless_msg)
        self.brushed_publisher.publish(brushed_msg)

def main(args=None):
    rclpy.init(args=args)
    firmware_node = sim_bridge_node()
    rclpy.spin(firmware_node)

if __name__ == "__main__":
    main()
        