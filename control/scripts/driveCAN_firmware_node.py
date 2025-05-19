import driveCANCommunication as dCAN
import rclpy
from rclpy.node import Node

class driveCan(Node):

    def __init__(self):
        super.__init__("drivecan_node")

        self.motorInfoPublisher = self.create_publisher(dict, "drive_motors_info", 10)

    def publish_motor_info():
        """input: None
            return: ['RF' : ['Voltage' : ..., 'Current' : ..., ...], ['RB' : ['Voltage' : ..., 'Current' : ..., ...], ['LF' : ['Voltage' : ..., 'Current' : ..., ...], ['LB' : ['Voltage' : ..., 'Current' : ..., ...]]
            Ask Aman if this is the ping thing
        """

    def publish_speeds(speeds -> list):
        #TODO: Check if all motors exist (James' func returns list of bools) && motors work and send speeds to motors and UI
