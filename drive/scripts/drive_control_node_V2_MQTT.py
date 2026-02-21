#!/usr/bin/env python3

#POTENTIAL ERRORS TO FIX:
# broadcast speed direction go # RF, LF, LB, RB
# steering angle direction go # RF, RB, LB, LF
#not sure if this is on purpose or a mistake

#imports from drive control node
import os
import sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
parent = currentdir.rfind("/", 0, currentdir.rfind("/")) # also add the top level folder as a path
sys.path.append(currentdir[:parent])

from speed_control import SpeedController
from steering import Steering
import math
import numpy as np
import driveCANCommunication as dCAN

# import rclpy
# from rclpy.node import Node
# from msg_srv_interface.msg import GamePadInput
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import Bool

# from msg_srv_interface.msg import DriveMotorDiagnostic
# from msg_srv_interface.srv import DriveMotorStatus


import json, time, socket
from time import time as now
from paho.mqtt.client import Client

BROKER = "localhost"
PORT = 1883
TOPIC = "rover/gamepad/drive"
QOS = 1
KEEPALIVE = 60

class drive_control_V2_MQTT:

    def __init__(self):

        #Declare field corresponding to speed control node and current state of wheels
        self.steering = Steering()

        # TODO: Tune values
        self.deadzone = 0.1 
        self.turning_speed = 3200.0
        
        self.tank_drive_mode = False
        
        #Call electrical API to get current state of wheels
        self.wheel_angles = [math.pi/2]*4 #Dummy  value, update with API call

        station              = dCAN.CANStation(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
        esc_interface        = dCAN.ESCInterface(station)
        self.drive_interface = dCAN.DriveInterface(esc_interface)

        self.nodes = [
            dCAN.NodeID.RF_DRIVE,
            dCAN.NodeID.RB_DRIVE,
            dCAN.NodeID.LB_DRIVE,
            dCAN.NodeID.LF_DRIVE
        ]

        self.motor_info = {node: {"Voltage": -1.0, "Current": -1.0, "State": -1.0, "Temperature": -1.0} for node in ["RF", "RB", "LB", "LF"]}
        #Steering motors should be appended to this dict.

        self.drive_speed_info = [0.0, 0.0, 0.0, 0.0] #List entries corresponding to [RF, RB, LB, LF]
        self.motors = list(self.motor_info.keys())
        self.pub_count = 0

        for motor in self.nodes:
            self.drive_interface.acknowledge_motor_fault(motor)

        #Firmware timer used to use 0.1s period, while control node timer uses 0.2s period
         # IMPORTANT: Timer period cannot be too high that it exceeds router buffer 
        #timer_period = 0.2
        #self.timer = self.create_timer(timer_period, self.run) 
    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker, rc=", rc)
        client.subscribe((TOPIC, QOS))
        client.publish("rover/drive/status", json.dumps({"status": "online"}), QOS)

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except Exception as e:
            print("Decode error:", e)
            return
        
        if msg.topic == TOPIC:
            self.handle_drive_input(data)

    def not_in_deadzone_check(self, x_axis: float, y_axis: float) -> bool:
        return not ((-self.deadzone <= x_axis <= self.deadzone) and (-self.deadzone <= y_axis <= self.deadzone)) 

    def handle_drive_input(self, data):
        acknowledge_msg = Bool()
        if data.get('square_button', 0.0):
            acknowledge_msg.data = True
            self.clear_motor_faults(acknowledge_msg)
        else:
            acknowledge_msg.data = False

        speed = self.steering.speed_controller.update_speed(data.get('x_button', 0.0), data.get('o_button', 0.0))
        speed = [speed for _ in range(4)]

        if data.get('r2_button', 0.0) or data.get('l2_button', 0.0):
            self.steering.speed_controller.shift_gear(
                data.get('r2_button', 0.0),
                data.get('l2_button', 0.0)
            )

        if data.get('r1_button', 0.0) or data.get('l1_button', 0.0):
            rot_inp = 1 if data.get('r1_button', 0.0) else -1
            rotation_sp = self.steering.rover_rotation(self.wheel_angles, rot_inp)
            speed = [direction*self.turning_speed for direction in rotation_sp]

        if data.get('triangle_button', 0.0):
            self.tank_drive_mode = not self.tank_drive_mode
            if self.tank_drive_mode:
                self.get_logger().info("TANK DRIVE MODE ACTIVATED - left stick controls left wheel & right stick controls right wheel")
            else:
                self.get_logger().info("TANK DRIVE MODE DEACTIVATED - left stick controls rover rotation")

        if self.tank_drive_mode:
            if self.not_in_deadzone_check(data.get('l_stick_x', 0.0), data.get('l_stick_y', 0.0)):
                left_speed_wheels = self.steering.update_left_wheel_speeds(data.get('l_stick_y', 0.0))
            else:
                left_speed_wheels = [0, 0]

            if self.not_in_deadzone_check(data.get('r_stick_x', 0.0), data.get('r_stick_y', 0.0)):
                right_speed_wheels = self.steering.update_right_wheel_speeds(data.get('r_stick_y', 0.0))
            else:
                right_speed_wheels = [0, 0]

            speed = [
                right_speed_wheels[0],
                left_speed_wheels[0],
                left_speed_wheels[1],
                right_speed_wheels[1]
            ]

            self.broadcast_speeds(speed)
            return

        if self.not_in_deadzone_check(data.get('l_stick_x', 0.0), data.get('l_stick_y', 0.0)):
            self.wheel_angles = self.steering.wheel_orientation_rot(
                data.get('l_stick_x', 0.0),
                data.get('l_stick_y', 0.0),
                self.wheel_angles[0]
            )
            self.broadcast_steering_angles(self.wheel_angles)

        self.broadcast_speeds(speed)


    # def controller_callback(self, input: GamePadInput):
    #     self.gamepad_input = input


    def clear_motor_faults(self, acknowledge_faults: bool): 
        if acknowledge_faults.data:
            for motor in self.nodes:
                self.drive_interface.acknowledge_motor_fault(motor)


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

    def publish_motor_info(self):
        self.update_motor_info()
        self.client.publish("rover/drive/motors_info", json.dumps({"timestamp": now(), "motors": self.motor_info}), qos = QOS)

    def publish_speed_info(self):
        self.update_speed_info()
        payload = {
            "timestamp": now(),
            "speeds": {
                "RF": self.drive_speed_info[0],
                "RB": self.drive_speed_info[1],
                "LB": self.drive_speed_info[2],
                "LF": self.drive_speed_info[3],
            },
        }
        self.client.publish("rover/drive/speeds_info", json.dumps(payload), qos = QOS)

    def broadcast_speeds(self, speeds):
        inp = [-speeds[0], speeds[1], -speeds[2], -speeds[3]]
        self.drive_interface.broadcast_multi_motor_speeds(inp)


    def broadcast_steering_angles(self, steering_angles):
        self.drive_interface.run_steer_motor_position(
            dCAN.NodeID.RF_STEER,
            steering_angles[0]
        )

    def drive_ping_callback(self):
        status_motors = self.drive_interface.getAllMotorStatus()
        payload = {
            "timestamp": now(),
            "rf_ok" : status_motors[0],
            "rb_ok" : status_motors[1],
            "lb_ok" : status_motors[2],
            "lf_ok" : status_motors[3]
        }
        
        self.client.publish("rover/drive/drive_motors_status", json.dumps(payload), qos=QOS)

    def loop(self):
        self.client = Client(client_id=f"gamepad_sub_{socket.gethostname()}")
        self.client.will_set(
            "rover/drive/status",
            payload=json.dumps({"status": "offline"}),
            qos=1, retain=False)

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(BROKER, PORT, keepalive=60)
        self.client.loop_start()

        try:
            while True:
                self.publish_motor_info()
                self.publish_speed_info()
                self.broadcast_speeds([0.0, 0.0, 0.0, 0.0])
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("Exiting publisher")
        finally:
            self.client.loop_stop()
            self.client.disconnect()

def main():
    drive_controller_V2_node = drive_control_V2_MQTT()
    drive_controller_V2_node.loop()

if __name__ == "__main__":
    main()