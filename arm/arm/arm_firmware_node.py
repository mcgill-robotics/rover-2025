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
from std_msgs.msg import Bool


class arm_firmware(Node):
    def __init__(self):
        super().__init__("arm_firmware_node")
        # Returns a list of waist, shoulder, elbow, wrist, hand
        self.position_subscriber = self.create_subscription(Float32MultiArray, "arm_position_cmd", self.broadcast_pos, 10)  
        self.faults_subscriber   = self.create_subscription(Bool, "acknowledge_arm_faults", self.clear_motor_faults, 10)
        
        self.positionFeedback_publisher = self.create_publisher(Float32MultiArray, "arm_position_feedback", 10)

        self.calibration_service = self.create_service(Trigger, "calibration_service", self.calibration_callback)

        self.station = aCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000) # ttyACM0 adapter for Linux, COM7 adapter on Windows
        #esc_interface = aCAN.ESCInterface(station) # armCANV1
        self.arm_interface = aCAN.ArmESCInterface(self.station) #armCANV2
        self.nodes = [aCAN.ArmNodeID.WAIST, aCAN.ArmNodeID.SHOULDER, aCAN.ArmNodeID.ELBOW]

    def clear_motor_faults(self, acknowledge_faults: Bool):
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.arm_interface.acknowledge_faults(motor)

    def broadcast_pos(self, position_cmd: Float32MultiArray):
        """ 
        Broadcast arm positions from arm_position_cmd 
        """
        data = position_cmd.data
        try:
            # self.arm_interface.run_follower(aCAN.ArmNodeID.WAIST, data[0])
            # time.sleep(0.025)
            # self.arm_interface.run_follower(aCAN.ArmNodeID.SHOULDER, data[1])
            # time.sleep(0.025)
            # self.arm_interface.run_follower(aCAN.ArmNodeID.ELBOW, data[2])
            # time.sleep(0.025)

            self.arm_interface.broadcast_follower(data[0], data[1], data[2]) # waist, shoulder, elbow
        except Exception as e:
            print(f"[Broadcasting pos] Error during broadcasting position: {e}")
    
            
    def calibration_callback(self, request, response):
        """
        Waits for calibration requests from calibration_service in arm_control_node and outputs 
        """
        print("[Calibration] Calibrating all joints...")
        try:
            # self.arm_interface.calibrate(aCAN.ArmNodeID.WAIST)
            # time.sleep(0.025)
            # self.arm_interface.calibrate(aCAN.ArmNodeID.SHOULDER)
            # time.sleep(0.025)
            # self.arm_interface.calibrate(aCAN.ArmNodeID.ELBOW)
            # time.sleep(0.025)

            joints = [aCAN.ArmNodeID.WAIST, aCAN.ArmNodeID.SHOULDER, aCAN.ArmNodeID.ELBOW]
            calibrate_blocking(self.arm_interface, self.arm_interface.station, joints)
            response.success = True
            print("Succesfully Calibrated")
            
        except Exception as e:
            print(f"[Calibration] Error during calibration: {e}")
            response.success = False

        finally:
            return response
        
    def read_position(self, joint: aCAN.ArmNodeID):
        position = self.arm_interface.read_position(joint)
        print(f"{joint} at position {position}")

def main(args=None):
    rclpy.init(args=args)
    node = arm_firmware()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
