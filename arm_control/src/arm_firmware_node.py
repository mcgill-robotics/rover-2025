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
        self.fault_acknowledgement_subscriber = self.create_subscription(Bool, "acknowledge_faults", self.clear_motor_faults, 10)

        station = aCAN.CANStation(interface="slcan", channel="COM6", birate=500000)
        esc_interface = aCAN.ESCInterface(station)
        self.arm_interface = aCAN.arm_interface(esc_interface)
        self.nodes = [aCAN.NodeID.WAIST, aCAN.NodeID.SHOULDER, aCAN.NodeID.ELBOW]


    # def run(self):
    #     self.publish_motor_position()
    #     speeds_msg = Float32MultiArray()
    #     speeds_msg.data = self.drive_speed_info
    #     self.drive_motors_speeds_publisher.publish(speeds_msg)

    #     if (self.pub_count % 5) == 0:
    #         self.publish_motor_info()
    #         self.pub_count = 0

    #     self.pub_count += 1

    def broadcast_post(self, position_cmd: Float32MultiArray):
        """
        Broadcasts the position command to the arm motors.
        """
        for i, node in enumerate(self.nodes):
            self.arm_interface.run_motor_position(node, position_cmd.data[i])
            # self.arm_interface.calibrate_motor(node)  # Uncomment to calibrate motor
            # self.arm_interface.acknowledge_faults(node)  # Uncomment to acknowledge faults
            # self.arm_interface.stop_motor(node)  # Uncomment to stop motor






