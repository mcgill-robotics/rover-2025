import driveCANCommunication as dCAN
import rclpy
from rclpy.node import Node

class driveCan(Node):

    def __init__(self):
        super.__init__("drivecan_node")

        self.motorInfoPublisher = self.create_publisher(dict, "drive_motors_info", 10)
        self.motorSpeedsPublisher = self.create_publisher(list, "drive_speeds_info", 10)

        station = dCAN.CANStation(interface="slcan", channel="can0", bitrate=500000)
        esc_interface = dCAN.ESCInterface(station)
        self.drive_interface = dCAN.DriveInterface(esc_interface)
        self.nodes = [dCAN.NodeID.RF_DRIVE, dCAN.NodeID.RB_DRIVE, dCAN.NodeID.LB_DRIVE, dCAN.NodeID.LF_DRIVE] #Steering motors should be appended
        self.motor_info = {"RF": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "RB": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "LB": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0},
                           "LF": {"Voltage": 0, "Current": 0, "Speed": 0, "State": 0, "Temperature": 0}}
        #Steering motors should be appended to this dict.
        self.motors = list(self.motor_info.keys())



    
    # def publish_motor_info(self, inp):
    #     """input: None
    #         return: ['RF' : ['Voltage' : ..., 'Current' : ..., ...], ['RB' : ['Voltage' : ..., 'Current' : ..., ...], ['LF' : ['Voltage' : ..., 'Current' : ..., ...], ['LB' : ['Voltage' : ..., 'Current' : ..., ...]]
    #         Ask Aman if this is the ping thing
    #     """
    #     if inp:
    #         nodes = [dCAN.NodeID.RF_DRIVE, dCAN.NodeID.RB_DRIVE, dCAN.NodeID.LB_DRIVE, dCAN.NodeID.LF_DRIVE] #Steering motors should be appended
    #         motors = list(self.motor_info.keys())
    #         states = self.drive_interface.getAllMotorStatus()

    #         for ind in range(len(nodes)):
    #             if states[ind]:
    #                 self.motor_info[motors[ind]]["Voltage"] = self.drive_interface.read_voltage(nodes[ind])
    #                 self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #                 self.motor_info[motors[ind]]["Current"] = self.drive_interface.read_current(nodes[ind])
    #                 self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #                 self.motor_info[motors[ind]]["Speed"] = self.drive_interface.read_speed(nodes[ind])
    #                 self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #                 self.motor_info[motors[ind]]["State"] = self.drive_interface.read_state(nodes[ind])
    #                 self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #                 self.motor_info[motors[ind]]["Temperature"] = self.drive_interface.read_temperature(nodes[ind])
    #                 self.drive_interface.esc.station.recv_msg(timeout=0.25)

    #     self.motorInfoPublisher.publish(self.motor_info)

    def update_voltage(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["Voltage"] = self.drive_interface.read_voltage(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)
        
        self.motorInfoPublisher.publish(self.motor_info)
        

    def update_current(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["Current"] = self.drive_interface.read_current(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)
        
        self.motorInfoPublisher.publish(self.motor_info)

    def update_speed(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["Speed"] = self.drive_interface.read_speed(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)
        
        self.motorInfoPublisher.publish(self.motor_info)


    def update_state(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["State"] = self.drive_interface.read_state(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)
        
        self.motorInfoPublisher.publish(self.motor_info)
    
    def update_temperature(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                self.motor_info[self.motors[ind]]["Temperature"] = self.drive_interface.read_temperature(self.nodes[ind])
                self.drive_interface.esc.station.recv_msg(timeout=0.25)
        
        self.motorInfoPublisher.publish(self.motor_info)
        

    def publish_speeds(self, speeds: list):
        #TODO: Check if all motors exist (James' func returns list of bools) && motors work and send speeds to motors and UI
        self.drive_interface.broadcast_multi_motor_speeds(speeds)
        self.motorSpeedsPublisher.publish(speeds)


def main(args=None):
    rclpy.init(args=args)
    driveCAN_node = driveCan()
    rclpy.spin(driveCAN_node)

if __name__ == "__main__":
    main()
