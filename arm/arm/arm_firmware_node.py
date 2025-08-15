import os
import sys
import time

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import armCANCommunicationV2 as aCAN
import rclpy
from allArmJointsv2 import calibrate_blocking
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

# from msg_srv_interface.msg import DriveMotorDiagnostic
# from msg_srv_interface.srv import DriveMotorStatus
from std_msgs.msg import Bool


class arm_firmware(Node):
    def __init__(self):
        super().__init__("arm_firmware_node")
        # Returns a list of wais,t shoulder, elbow, wrist, hand
        self.position_subscriber = self.create_subscription(Float32MultiArray, "arm_position_cmd", self.broadcast_pos, 10)  
        self.faults_subscriber   = self.create_subscription(Bool, "acknowledge_arm_faults", self.clear_motor_faults, 10)

        self.calibration_service = self.create_service(Trigger, "calibration_service", self.calibration_callback)

        station = aCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        #esc_interface = aCAN.ESCInterface(station) # armCANV1
        self.arm_interface = aCAN.ArmESCInterface(station) #armCANV2
        self.nodes = [aCAN.ArmNodeID.WAIST, aCAN.ArmNodeID.SHOULDER, aCAN.ArmNodeID.ELBOW]

    def clear_motor_faults(self, acknowledge_faults: Bool):
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.arm_interface.acknowledge_faults(motor)

    def broadcast_pos(self, position_cmd: Float32MultiArray):
        #Old V1 Code:
        #for i, node in enumerate(self.nodes):
            #self.arm_interface.run_motor_position(node, position_cmd.data[i])
            # self.arm_interface.calibrate_motor(node)  # Uncomment to calibrate motor
            # self.arm_interface.acknowledge_faults(node)  # Uncomment to acknowledge faults
            # self.arm_interface.stop_motor(node)  # Uncomment to stop motor

        data = position_cmd.data
        self.arm_interface.broadcast_positions(data[0], data[1], data[2])  # Joint angles for   

    def calibration_callback(self, request, response):
        print("[Calibration] Calibrating all joints...")
        try:
            # self.arm_interface.calibrate(aCAN.ArmNodeID.WAIST)
            # time.sleep(0.025)

            # self.arm_interface.calibrate(aCAN.ArmNodeID.SHOULDER)
            # time.sleep(0.025)

            # self.arm_interface.calibrate(aCAN.ArmNodeID.ELBOW)
            # time.sleep(0.025)

            

            joints = [aCAN.ArmNodeID.SHOULDER, aCAN.ArmNodeID.ELBOW]
            calibrate_blocking(self.arm_interface, self.arm_interface.station, joints)
            response.success = True
            print("Succesfully Calibrated")


        except Exception as e:
            print(f"[Calibration] Error during calibration: {e}")
            response.success = False

        finally:
            return response

def main(args=None):
    rclpy.init(args=args)
    node = arm_firmware()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
