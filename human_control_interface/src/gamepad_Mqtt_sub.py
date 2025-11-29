#!/usr/bin/env python3
import json, socket
from paho.mqtt.client import Client

BROKER = "localhost"
PORT = 1883
TOPIC_DRIVE = "rover/gamepad/drive"
TOPIC_ARM   = "rover/gamepad/arm"
QOS = 1

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker, rc=", rc)
    client.subscribe([(TOPIC_DRIVE, QOS), (TOPIC_ARM, QOS)])

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
    except Exception as e:
        print("Decode error:", e)
        return

    if msg.topic == TOPIC_DRIVE:
        handle_drive_input(data)
    elif msg.topic == TOPIC_ARM:
        handle_arm_input(data)

def handle_drive_input(data):
    # Example mapping, we need to integrate to the actual drive code
    l_x = data.get('l_stick_x', 0.0)
    l_y = data.get('l_stick_y', 0.0)
    print(f"Drive: Lx={l_x:.2f}, Ly={l_y:.2f}")

def handle_arm_input(data):
    # Example mapping, we need to integrate to the actual arm code
    print(f"Arm: Lx={data.get('l_stick_x')}, Ly={data.get('l_stick_y')}")

def main():
    client = Client(client_id=f"gamepad_sub_{socket.gethostname()}")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_forever()

if __name__ == "__main__":
    main()
