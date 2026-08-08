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
from stitching import Stitcher
from msg_srv_interface.srv import PanTiltSavePhoto
import cv2
from pathlib import Path
import shutil

BROKER = "localhost" # Change to MQTT Broker IP address
PORT = 1883
TOPIC = "rover/gamepad/drive"
QOS = 1
KEEPALIVE = 60
PANTILT_IMAGE_FOLDER = "INSERT FOLDER HERE" # TODO
PANORAMA_FOLDER = "INSERT FOLDER HERE" # TODO
PANTILT_SIGNAL_FILE = "INSERT ABSOLUTE PATH HERE" # TODO

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
        self.pantilt_firmware = pf.PanTiltGPS(f"{get_ACM_port(subsystem = Subsystem.GPS)}")
        try:
            self.pantilt_firmware.connect()
        except:
            self.get_logger().error("Failed to connect to the PanTiltGPS board. Check the connection.")
            return
        self.step_size = 5 #in degrees
        timer_period = 1e-2
        self.photo_client = self.create_client(PanTiltSavePhoto, 'save-photo')

        self.gps_publisher = self.create_publisher(Float32MultiArray, "roverGPSData", 10)
        # self.imu_publisher = self.create_publisher(Float32MultiArray, "roverIMUData", 10)

        self.timer = self.create_timer(timer_period, self.run)

        self.remaining_panoramic_steps = 0
        
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
        pan_button = data.get("select_button", False)
        if(pan_button):
            self.remaining_panoramic_steps = 9 # PLAY AROUND WITH THIS
            self.pan_step_size = 180/self.remaining_panoramic_steps
            # if (self.pantilt_firmware.get_pantilt()[0]>180): #TODO: is 360 the limit?
                # pan_change = 180-self.pantilt_firmware.get_pantilt[0] #move pantilt to a viable position
            # delete previous images
            
        elif self.remaining_panoramic_steps > 0: # placeholder, starts panoramic picture
            tilt_change = 0
            pan_change = self.pan_step_size # TODO: TEST THIS
            self.remaining_panoramic_steps -= 1

            request = PanTiltSavePhoto.Request()

            if self.remaining_panoramic_steps == 0:
                request.make_pano = True
            else:
                request.make_pano = False

            self.photo_client.wait_for_service()
            self.future = self.photo_client.call_async(request)
            rclpy.spin_until_future_complete(self, self.future)
            
        else:
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