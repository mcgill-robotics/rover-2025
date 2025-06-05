import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import driveCANCommunication as dCAN
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg_srv_interface.msg import GamePadInput
from msg_interface.msg import DriveMotorDiagnostic, DriveMotorStatus
#import can

class driveCan(Node):

    def __init__(self):
        super().__init__("drivecan_node")

        self.driveSpeedInputSubscriber = self.create_subscription(Float32MultiArray, "drive_speed_input", self.broadcast_speeds, 10)
        self.gampepad_subscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.clear_motor_faults, 10)

        self.driveMotorsInfoPublisher = self.create_publisher(DriveMotorDiagnostic, "drive_motors_info", 10) 

        self.driveMotorsSpeedsPublisher = self.create_publisher(Float32MultiArray, "drive_speeds_info", 10)
        
        self.drivePingService = self.create_service(DriveMotorStatus, "drive_motors_status", self.drive_ping_callback)

        station = dCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        esc_interface = dCAN.ESCInterface(station)
        self.drive_interface = dCAN.DriveInterface(esc_interface)
        self.nodes = [dCAN.NodeID.RF_DRIVE, dCAN.NodeID.RB_DRIVE, dCAN.NodeID.LB_DRIVE, dCAN.NodeID.LF_DRIVE] #Steering motors should be appended
        self.motor_info = {"RF": {"Voltage": 0, "Current": 0, "State": 0, "Temperature": 0},
                           "RB": {"Voltage": 0, "Current": 0, "State": 0, "Temperature": 0},
                           "LB": {"Voltage": 0, "Current": 0, "State": 0, "Temperature": 0},
                           "LF": {"Voltage": 0, "Current": 0, "State": 0, "Temperature": 0}}
        #Steering motors should be appended to this dict.

        self.drive_speed_info = [0,0,0,0] #List entries corresponding to [RF, RB, LB, LF]
        self.motors = list(self.motor_info.keys())
        self.pub_count = 0

        for motor in self.nodes:
            self.drive_interface.acknowledge_motor_fault(motor)

        # TODO: Uncomment when custom msg for dict is done
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    def run(self):
        self.update_speed_info()
        speeds_msg = Float32MultiArray()
        speeds_msg.data = self.drive_speed_info
        self.driveMotorsSpeedsPublisher.publish(speeds_msg)

        if (self.pub_count % 5) == 0:
            self.publish_motor_info()
            self.pub_count = 0

        self.pub_count += 1


    def clear_motor_faults(self, gamepad_input : GamePadInput):

        if gamepad_input.square_button:
            for motor in self.nodes:
                self.drive_interface.acknowledge_motor_fault(motor)

    # TODO: Test publish motor_info with UI
    def update_motor_info(self):
        states = self.drive_interface.getAllMotorStatus()

        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["Voltage"] = self.drive_interface.read_voltage(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)

                self.motor_info[self.motors[ind]]["Current"] = self.drive_interface.read_current(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)

                self.motor_info[self.motors[ind]]["State"] = self.drive_interface.read_state(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)

                self.motor_info[self.motors[ind]]["Temperature"] = self.drive_interface.read_temperature(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)

            else:
                print("Motor fault detected in " + self.motors[ind])
                exit(1)

    def publish_motor_info(self):
        self.update_motor_info()
        msg = DriveMotorDiagnostic()

        #Fill msg values with diagnostic info from the dictionary
        #RF
        msg.RF_Voltage = self.motor_info["RF"]["Voltage"]
        msg.RF_Current = self.motor_info["RF"]["Current"]
        msg.RF_State = self.motor_info["RF"]["State"]
        msg.RF_Temperature = self.motor_info["RF"]["Temperature"]

        #RB
        msg.RB_Voltage = self.motor_info["RB"]["Voltage"]
        msg.RB_Current = self.motor_info["RB"]["Current"]
        msg.RB_State = self.motor_info["RB"]["State"]
        msg.RB_Temperature = self.motor_info["RB"]["Temperature"]

        #LB
        msg.LB_Voltage = self.motor_info["LB"]["Voltage"]
        msg.LB_Current = self.motor_info["LB"]["Current"]
        msg.LB_State = self.motor_info["LB"]["State"]
        msg.LB_Temperature = self.motor_info["LB"]["Temperature"]

        #LF
        msg.LF_Voltage = self.motor_info["LF"]["Voltage"]
        msg.LF_Current = self.motor_info["LF"]["Current"]
        msg.LF_State = self.motor_info["LF"]["State"]
        msg.LF_Temperature = self.motor_info["LF"]["Temperature"]

        self.driveMotorsInfoPublisher.publish(msg)

              

    def update_speed_info(self):
        states = self.drive_interface.getAllMotorStatus()

        for ind in range(len(self.nodes)):
            if states[ind]:
                self.drive_speed_info[ind] = self.drive_interface.read_speed(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)

            else:
                print("Motor fault detected in " + self.motors[ind])
                exit(1)

    
    def broadcast_speeds(self, speeds: Float32MultiArray):
        inp = speeds.data
        self.drive_interface.broadcast_multi_motor_speeds(inp)


    '''request: contains the request data
       response: empty response object that is filled with the response data'''
    def drive_ping_callback(self, request, response):
        status_motors = self.drive_interface.getAllMotorStatus()
        response.RF_OK = status_motors[0]
        response.RB_OK = status_motors[1]
        response.LB_OK = status_motors[2]
        response.LF_OK = status_motors[3]
        return response


def main(args=None):
    rclpy.init(args=args)
    driveCAN_node = driveCan()
    rclpy.spin(driveCAN_node)

if __name__ == "__main__":
    main()
