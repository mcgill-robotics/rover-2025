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

    def publish_motor_info():
        """input: None
            return: ['RF' : ['Voltage' : ..., 'Current' : ..., ...], ['RB' : ['Voltage' : ..., 'Current' : ..., ...], ['LF' : ['Voltage' : ..., 'Current' : ..., ...], ['LB' : ['Voltage' : ..., 'Current' : ..., ...]]
            Ask Aman if this is the ping thing
        """

    def publish_speeds(self, speeds: list):
        #TODO: Check if all motors exist (James' func returns list of bools) && motors work and send speeds to motors and UI
        speedList = self.drive_interface.broadcast_multi_motor_speeds(speeds)
        self.motorSpeedsPublisher.publish(speedList)
