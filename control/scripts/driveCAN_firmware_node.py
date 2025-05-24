import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import driveCANCommunication as dCAN
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg_srv_interface.msg import GamePadInput
import can

class driveCan(Node):

    def __init__(self):
        super().__init__("drivecan_node")

        self.driveSpeedInputSubscriber = self.create_subscription(Float32MultiArray, "drive_speed_input", self.publish_speeds, 10)
        self.gampepad_subscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.clear_motor_faults, 10)

        #TODO: Create custom msg type to send the dictionary
        #self.motorInfoPublisher = self.create_publisher(dict, "drive_motors_info", 10) 

        self.motorSpeedsPublisher = self.create_publisher(Float32MultiArray, "drive_speeds_info", 10)
        
        #TODO: Create custom .srv
        # self.drivePingService = self.create_service(Float32MultiArray, "drive_motors_status", self.drive_ping_callback)

        station = dCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        esc_interface = dCAN.ESCInterface(station)
        self.drive_interface = dCAN.DriveInterface(esc_interface)
        self.nodes = [dCAN.NodeID.RF_DRIVE, dCAN.NodeID.RB_DRIVE, dCAN.NodeID.LB_DRIVE, dCAN.NodeID.LF_DRIVE] #Steering motors should be appended
        self.motor_info = {"RF": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "RB": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "LB": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "LF": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0}}
        #Steering motors should be appended to this dict.
        self.motors = list(self.motor_info.keys())

        for motor in self.nodes:
            self.drive_interface.acknowledge_motor_fault(motor)

        #TODO: Uncomment when custom msg for dict is done
        # timer_period = 5.0
        # self.timer = self.create_timer(timer_period, self.publish_motor_info)

    def clear_motor_faults(self, gamepad_input : GamePadInput):

        if gamepad_input.square_button:
            for motor in self.nodes:
                self.get_logger().info('Hello')
                self.drive_interface.acknowledge_motor_fault(motor)



    
    # def publish_motor_info(self):
    #     states = self.drive_interface.getAllMotorStatus()

    #     for ind in range(len(self.nodes)):
    #         if states[ind]:
    #             self.motor_info[self.motors[ind]]["Voltage"] = self.drive_interface.read_voltage(self.nodes[ind])
    #             self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #             self.motor_info[self.motors[ind]]["Current"] = self.drive_interface.read_current(self.nodes[ind])
    #             self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #             self.motor_info[self.motors[ind]]["Speed"] = self.drive_interface.read_speed(self.nodes[ind])
    #             self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #             self.motor_info[self.motors[ind]]["State"] = self.drive_interface.read_state(self.nodes[ind])
    #             self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #             self.motor_info[self.motors[ind]]["Temperature"] = self.drive_interface.read_temperature(self.nodes[ind])
    #             self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #         else:
    #             print("Motor fault detected in " + self.motors[ind])
    #             exit(1)

    #     self.motorInfoPublisher.publish(self.motor_info)

        

    def publish_speeds(self, speeds: Float32MultiArray):
        states = self.drive_interface.getAllMotorStatus()
        ok = True
        count = 0
        # for cond in states:
        #     if not cond:
        #         ok = False
        #         break
        #     count += 1
        
        # if ok:
        inp = speeds.data
        inp[1] = -inp[1]
        inp[2] = -inp[2]
        self.drive_interface.broadcast_multi_motor_speeds(inp)
        self.motorSpeedsPublisher.publish(speeds)

        # else:
        #     print("Motor fault detected in " + self.motors[count])
        #     exit(1)

    #TODO: Create custom srv
    # def drive_ping_callback(self):
    #     states = self.drive_interface.getAllMotorStatus()
    #     msg = Float32MultiArray()
    #     msg.data = states
    #     return msg


def main(args=None):
    rclpy.init(args=args)
    driveCAN_node = driveCan()
    rclpy.spin(driveCAN_node)

if __name__ == "__main__":
    main()
