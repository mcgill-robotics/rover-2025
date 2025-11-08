#!/usr/bin/env python3

#imports from drive control node
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rclpy
from speed_control import SpeedController
from rclpy.node import Node
from msg_srv_interface.msg import GamePadInput
from steering import Steering
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

#imports from firmwear node
import driveCANCommunication as dCAN
from msg_srv_interface.msg import DriveMotorDiagnostic
from msg_srv_interface.srv import DriveMotorStatus


class drive_control_V2(Node):
    def __init__(self):
        super().__init__("drive_control_firmware_node")

        #Declare fields corresponging to controller input
        self.gamepad_input = GamePadInput()

        #Declare field corresponding to speed control node and current state of wheels
        self.steering = Steering()

        # TODO: Tune values
        self.deadzone = 0.1 
        self.turning_speed = 3200.0

        self.tank_drive_mode = False

        #Call electrical API to get current state of wheels
        self.wheel_angles = [math.pi/2]*4 #Dummy  value, update with API call

        self.gamepad_subscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.controller_callback, 10)

        #publishers and service from firmware node
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

        #Firmware timer used to use 0.1s period, while control node timer uses 0.2s period
         # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.run)



    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone))
   
    def run(self):
        #COMMENTS FROM FIRMWARE NODE, CAN PROBABLY BE REMOVED
        # self.update_speed_info()
        # speeds_msg = Float32MultiArray()
        # speeds_msg.data = self.drive_speed_info
        # self.drive_motors_speeds_publisher.publish(speeds_msg)

        # if (self.pub_count % 5) == 0:
        #     self.publish_motor_info()
        #     self.pub_count = 0

        # self.pub_count += 1
       

        #square -> acknowledged the faults, wheels will stop taking command until it is acknolewdged
        acknowledge_msg = Bool()
        if self.gamepad_input.square_button:
            acknowledge_msg.data = True

            #CHANGES MADE HERE 

            #instead of publishing here, call clear_motor_faults directly
            #self.acknowledgement_publisher.publish(acknowledge_msg) 
            
            self.clear_motor_faults(acknowledge_msg) 

            #END CHANGES 
        else:
            acknowledge_msg.data = False

            

            #Speed given button input
        speed = self.steering.speed_controller.update_speed(self.gamepad_input.x_button, self.gamepad_input.o_button)
        speed = [speed for _ in range(4)]

        #Check whether gears change
        if self.gamepad_input.r2_button or self.gamepad_input.l2_button:
            self.steering.speed_controller.shift_gear(self.gamepad_input.r2_button, self.gamepad_input.l2_button)

        #Check whether there is an input value for rover rotation
        if self.gamepad_input.r1_button or self.gamepad_input.l1_button:
            rot_inp = 0
            if (self.gamepad_input.r1_button):
                rot_inp = 1
            elif (self.gamepad_input.l1_button):
                rot_inp = -1

            #Array with the desired speed for each wheel during rover rotation
            rotation_sp = self.steering.rover_rotation(self.wheel_angles, rot_inp)

            speed = [direction*self.turning_speed for direction in rotation_sp]

            # Checking for tank drive mode
        if self.gamepad_input.triangle_button:
            self.tank_drive_mode = not self.tank_drive_mode
            if self.tank_drive_mode:
                self.get_logger().info("TANK DRIVE MODE ACTIVATED - left stick controls left wheel & right stick controls right wheel")
            else:
                self.get_logger().info("TANK DRIVE MODE DEACTIVATED - left stick controls rover rotation")
            
        if self.tank_drive_mode:    
            if self.not_in_deadzone_check(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y):
                left_speed_wheels = self.steering.update_left_wheel_speeds(self.gamepad_input.l_stick_y)
            else:
                left_speed_wheels = [0, 0]
            if self.not_in_deadzone_check(self.gamepad_input.r_stick_x, self.gamepad_input.r_stick_y):
                right_speed_wheels = self.steering.update_right_wheel_speeds(self.gamepad_input.r_stick_y)
            else:
                right_speed_wheels = [0, 0]
                
            speed = [right_speed_wheels[0], left_speed_wheels[0], left_speed_wheels[1], right_speed_wheels[1]]
            msg = Float32MultiArray()
            msg.data = [float(s) for s in speed]
            print(msg.data)

            #CHANGES MADE HERE
            self.broadcast_speeds(msg)
            #self.speed_input_publisher.publish(msg)  
           # END CHANGES
        
            return
        
        #Check whether joystick position changes
        if self.not_in_deadzone_check(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y):
            #Orientation for wheels given a joystick positinon
            self.wheel_angles = self.steering.wheel_orientation_rot(self.gamepad_input.l_stick_x, self.gamepad_input.l_stick_y, self.wheel_angles[0])

            # TODO: Send new orientation to wheels
            msg = Float32MultiArray()
            msg.data = [float(angle) for angle in self.wheel_angles]

            #CHANGES MADE HERE
            self.broadcast_steering_angles(msg)
            #self.steering_input_publisher.publish(msg) 
            #END CHANGES

        # TODO: Use API to send input values for speed, orientation and rotation.
        msg = Float32MultiArray()
        msg.data = [float(s) for s in speed]

        #CHANGES MADE HERE
        self.broadcast_speeds(msg)
        #self.speed_input_publisher.publish(msg)
        #END CHANGES

    def controller_callback(self, input: GamePadInput):
        self.gamepad_input = input


    #Adding in the functions from firmware node that are called in the control node via publishers

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
        inp = [-speeds.data[0], speeds.data[1], -speeds.data[2], -speeds.data[3]] # RF, LF, LB, RB
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
    drive_controller_V2_node = drive_control_V2()
    rclpy.spin(drive_controller_V2_node)

if __name__ == "__main__":
    main()