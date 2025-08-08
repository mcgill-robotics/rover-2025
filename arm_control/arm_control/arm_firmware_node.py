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


class arm_firmware(Node):
    def __init__(self):
        super().__init__("arm_firmware_node")
        # Returns a list of wais,t shoulder, elbow, wrist, hand
        self.position_subscriber = self.create_subscription(Float32MultiArray, "arm_position_cmd", self.broadcast_pos, 10)  
        self.faults_subscriber   = self.create_subscription(Bool, "acknowledge_arm_faults", self.clear_motor_faults, 10)

        station = aCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        esc_interface = aCAN.ESCInterface(station)
        self.arm_interface = aCAN.SystemInterface(esc_interface, aCAN.MotorType.STEER)
        self.nodes = [aCAN.NodeID.WAIST, aCAN.NodeID.SHOULDER, aCAN.NodeID.ELBOW]

    def clear_motor_faults(self, acknowledge_faults: Bool):
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.arm_interface.acknowledge_faults(motor)

    def broadcast_pos(self, position_cmd: Float32MultiArray):
        for i, node in enumerate(self.nodes):
            self.arm_interface.run_motor_position(node, position_cmd.data[i])
            # self.arm_interface.calibrate_motor(node)  # Uncomment to calibrate motor
            # self.arm_interface.acknowledge_faults(node)  # Uncomment to acknowledge faults
            # self.arm_interface.stop_motor(node)  # Uncomment to stop motor


def main(args=None):
    rclpy.init(args=args)
    node = arm_firmware()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
