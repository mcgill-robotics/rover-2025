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


        # get shit from arm_control_node 
        # arm_control_cmd

class arm_firmware(Node):
    def __init__(self):
        super.__init__("arm_firmware_node")
        self.position_subscriber = self.create_subscription(Float32MultiArray, "arm_position_cmd", self.broadcast_pos, 10) # Returns a list of wais,t shoulder, elbow, wrist, hand
        self.faults_subscriber = self.create_subscription(Bool, "acknowledge_arm_faults", self.clear_motor_faults, 10)

        station = aCAN.CANStation(interface="slcan", channel="COM6", birate=500000)
        esc_interface = aCAN.ESCInterface(station)
        self.arm_interface = aCAN.arm_interface(esc_interface)
        self.nodes = [aCAN.NodeID.WAIST, aCAN.NodeID.SHOULDER, aCAN.NodeID.ELBOW]

    def clear_motor_faults(self, acknowledge_faults: Bool):
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.arm_interfaces.acknowledge_motor_fault(motor)


