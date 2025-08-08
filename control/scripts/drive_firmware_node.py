import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import driveCANCommunication as dCAN
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg_srv_interface.msg import DriveMotorDiagnostic
from msg_srv_interface.srv import DriveMotorStatus
from std_msgs.msg import Bool


class drive_firmware(Node):

    def __init__(self):
        super().__init__("drive_firmware_node")

        self.drive_speed_input_subscriber     = self.create_subscription(Float32MultiArray, "drive_speed_input",   self.broadcast_speeds,   10)
        self.drive_steering_input_subscriber  = self.create_subscription(Float32MultiArray, "drive_steering_input",   self.broadcast_steering_angles,   10)
        self.fault_acknowledgement_subscriber = self.create_subscription(Bool, "acknowledge_faults", self.clear_motor_faults, 10)
        self.drive_motors_info_publisher   = self.create_publisher(DriveMotorDiagnostic, "drive_motors_info", 10) 
        self.drive_motors_speeds_publisher = self.create_publisher(Float32MultiArray,    "drive_speeds_info", 10)
        self.drive_ping_service            = self.create_service(DriveMotorStatus,       "drive_motors_status", self.drive_ping_callback)

        station              = dCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        esc_interface        = dCAN.ESCInterface(station)
        self.drive_interface = dCAN.DriveInterface(esc_interface)
        self.nodes           = [dCAN.NodeID.RF_DRIVE, dCAN.NodeID.RB_DRIVE, dCAN.NodeID.LB_DRIVE, dCAN.NodeID.LF_DRIVE] #Steering motors should be appended

        self.motor_info = {node: {"Voltage": -1.0, "Current": -1.0, "State": -1.0, "Temperature": -1.0} for node in ["RF", "RB", "LB", "LF"]}
        #Steering motors should be appended to this dict.

        self.drive_speed_info = [0.0, 0.0, 0.0, 0.0] #List entries corresponding to [RF, RB, LB, LF]
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
        self.drive_motors_speeds_publisher.publish(speeds_msg)

        if (self.pub_count % 5) == 0:
            self.publish_motor_info()
            self.pub_count = 0

        self.pub_count += 1
        pass


    def clear_motor_faults(self, acknowledge_faults: Bool):
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.drive_interface.acknowledge_motor_fault(motor)

    # TODO: Test publish motor_info with UI
    def update_motor_info(self):
        states = self.drive_interface.getAllMotorStatus()

        for ind in range(len(self.nodes)):
            if states[ind]:
                try:
                    info = "Voltage" 
                    self.drive_interface.read_voltage(self.nodes[ind])
                    self.motor_info[self.motors[ind]]["Voltage"] = self.drive_interface.esc.station.recv_msg(timeout=0.25)[2]

                    info = "Current"
                    self.drive_interface.read_current(self.nodes[ind])
                    self.motor_info[self.motors[ind]]["Current"] = self.drive_interface.esc.station.recv_msg(timeout=0.25)[2]

                    info = "State"
                    self.drive_interface.read_state(self.nodes[ind])
                    self.motor_info[self.motors[ind]]["State"] = self.drive_interface.esc.station.recv_msg(timeout=0.25)[2]

                    #Uncomment when able to get temperature readings (July. 28 2025)
                    #info = "Temperature"
                    #self.drive_interface.read_temperature(self.nodes[ind])
                    #self.motor_info[self.motors[ind]]["Temperature"] = self.drive_interface.esc.station.recv_msg(timeout=0.25)[2]

                except:
                    print("Unable to get " + info + " on " + self.motors[ind])
                    self.motor_info[self.motors[ind]]["Voltage"] = -1.0
                    self.motor_info[self.motors[ind]]["Current"] = -1.0
                    self.motor_info[self.motors[ind]]["State"] = -1.0
                    self.motor_info[self.motors[ind]]["Temperature"] = -1.0
            else:
                self.motor_info[self.motors[ind]]["Voltage"] = -1.0
                self.motor_info[self.motors[ind]]["Current"] = -1.0
                self.motor_info[self.motors[ind]]["State"] = -1.0
                self.motor_info[self.motors[ind]]["Temperature"] = -1.0

    def publish_motor_info(self):
        self.update_motor_info()
        msg = DriveMotorDiagnostic()

        #Fill msg values with diagnostic info from the dictionary
        #RF
        msg.rf_voltage = self.motor_info["RF"]["Voltage"]
        msg.rf_current = self.motor_info["RF"]["Current"]
        msg.rf_state = self.motor_info["RF"]["State"]
        msg.rf_temperature = self.motor_info["RF"]["Temperature"]

        #RB
        msg.rb_voltage = self.motor_info["RB"]["Voltage"]
        msg.rb_current = self.motor_info["RB"]["Current"]
        msg.rb_state = self.motor_info["RB"]["State"]
        msg.rb_temperature = self.motor_info["RB"]["Temperature"]

        #LB
        msg.lb_voltage = self.motor_info["LB"]["Voltage"]
        msg.lb_current = self.motor_info["LB"]["Current"]
        msg.lb_state = self.motor_info["LB"]["State"]
        msg.lb_temperature = self.motor_info["LB"]["Temperature"]

        #LF
        msg.lf_voltage = self.motor_info["LF"]["Voltage"]
        msg.lf_current = self.motor_info["LF"]["Current"]
        msg.lf_state = self.motor_info["LF"]["State"]
        msg.lf_temperature = self.motor_info["LF"]["Temperature"]

        self.drive_motors_info_publisher.publish(msg)

    def update_speed_info(self):
        states = self.drive_interface.getAllMotorStatus()
        for ind in range(len(self.nodes)):
            if states[ind]:
                try:
                    self.drive_interface.read_speed(self.nodes[ind])
                    self.drive_speed_info[ind] = self.drive_interface.esc.station.recv_msg(timeout=0.25)[2]
                
                except:
                    print("Unable to get Speed on " + self.motors[ind])
                    self.drive_speed_info[ind] = -1.0
            else:
                self.drive_speed_info[ind] = -1.0

    
    def broadcast_speeds(self, speeds: Float32MultiArray):
        inp = [speeds.data[0], speeds.data[1], -speeds.data[2], -speeds.data[3]] # RF, LF, LB, RB
        self.drive_interface.broadcast_multi_motor_speeds(inp)

    def broadcast_steering_angles(self, steering_angles: Float32MultiArray):
        inp = steering_angles.data[0]
        self.drive_interface.run_steer_motor_position(dCAN.NodeID.RF_STEER, inp)


    '''request: contains the request data
       response: empty response object that is filled with the response data'''
    def drive_ping_callback(self, request, response):
        status_motors = self.drive_interface.getAllMotorStatus()
        response.rf_ok = status_motors[0]
        response.rb_ok = status_motors[1]
        response.lb_ok = status_motors[2]
        response.lf_ok = status_motors[3]
        return response

def main(args=None):
    rclpy.init(args=args)
    drive_firmware_node = drive_firmware()
    rclpy.spin(drive_firmware_node)

if __name__ == "__main__":
    main()
