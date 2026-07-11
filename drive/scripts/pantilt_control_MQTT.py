import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
parent = currentdir.rfind("/", 0, currentdir.rfind("/")) # also add the top level folder as a path
sys.path.append(currentdir[:parent])

import pantilt_firmware as pf
import rclpy
import json, time, socket
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from utils.get_acm_port import get_ACM_port, Subsystem
from paho.mqtt.client import Client

BROKER = "localhost" # Change to MQTT Broker IP address
PORT = 1883
TOPIC = "rover/gamepad/drive"
QOS = 1
KEEPALIVE = 60

class pantilt(Node):

    def __init__(self):
        super().__init__("pantilt_node")

        # MQTT structure
        self.client = Client(client_id=f"pantilt_gamepad_sub_{socket.gethostname()}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, keepalive=60)
        self.client.loop_start()

        # ROS structure
        # self.gampepad_subscriber = self.create_subscription(GamePadInput, "gamepad_input_drive", self.update_pantilt, 10)
        self.pantilt_firmware = pf.PanTiltGPS(f"/dev/ttyACM{get_ACM_port(subsystem = Subsystem.GPS)}")
        try:
            self.pantilt_firmware.connect()
        except:
            self.get_logger().error("Failed to connect to the PanTiltGPS board. Check the connection.")
            return
        self.step_size = 5 #in degrees
        timer_period = 1e-2

        self.gps_publisher = self.create_publisher(Float32MultiArray, "roverGPSData", 10)
        # self.imu_publisher = self.create_publisher(Float32MultiArray, "roverIMUData", 10)

        self.timer = self.create_timer(timer_period, self.run)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker, rc=", rc)
        client.subscribe((TOPIC, QOS))

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
        except Exception as e:
            print("Decode error:", e)
            return
        
        if msg.topic == TOPIC:
            self.update_pantilt(data)

    def run(self):
        # This method calls the run() method of the firmware, and sends the GPS data its respective topic.
        self.pantilt_firmware.run()
        gps_data = self.pantilt_firmware.get_gps()
        gps_msg = Float32MultiArray()
        gps_msg.data = tuple(gps_data)
        self.gps_publisher.publish(gps_msg)


    def update_pantilt(self, data) :
        # Update the angles based on the gamepad input
        inp_x = data.get("d_pad_x", 0.0)
        inp_y = data.get("d_pad_y", 0.0)
        # Debugging:
        print("INP_X: " + str(inp_x))
        print("INP_Y: " + str(inp_y))
        tilt_change = -inp_x * self.step_size # NOTE: Negated the input to ensure tilt up and down moved the camera accordingly
        pan_change = inp_y * self.step_size
        # Control the servos
        try:
            self.pantilt_firmware.add_pan_angle(pan_change)
            self.pantilt_firmware.add_tilt_angle(tilt_change)
        except ConnectionError as e:
            self.get_logger().error(f"Failed to update pan/tilt angles: {e}")
            return
        

def main(args=None):
    rclpy.init(args=args)
    pantilt_node = pantilt()
    rclpy.spin(pantilt_node)

if __name__ == "__main__":
    main()