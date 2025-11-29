#!/usr/bin/env python3
import json, time, socket, sys
from time import time as now
from paho.mqtt.client import Client
from gamepad import Gamepad

BROKER = "JETSON_IP_OR_BROKER_IP"
PORT = 1883
QOS = 1
KEEPALIVE = 60
TOPIC_DRIVE = "rover/gamepad/drive"
TOPIC_ARM   = "rover/gamepad/arm"

def gp_to_dict(gp):
    return {
        "timestamp": now(),
        "x_button": int(gp.b1),
        "o_button": int(gp.b2),
        "triangle_button": int(gp.b3),
        "square_button": int(gp.b4),
        "l1_button": int(gp.b5),
        "r1_button": int(gp.b6),
        "l2_button": int(gp.b7),
        "r2_button": int(gp.b8),
        "select_button": int(gp.b9),
        "start_button": int(gp.b10),
        "home_button": int(gp.b11),
        "l3_button": int(gp.b12),
        "r3_button": int(gp.b13),
        "l_stick_x": float(gp.a1),
        "l_stick_y": float(gp.a2),
        "l_stick_analog": float(gp.a3),
        "r_stick_x": float(gp.a4),
        "r_stick_y": float(gp.a5),
        "r_stick_analog": float(gp.a6),
        "d_pad_x": float(gp.a7[0]),
        "d_pad_y": float(gp.a7[1])
    }

def main():
    try:
        gp = Gamepad()
    except Exception as e:
        print("Controller not found:", e)
        sys.exit(1)

    mqttc = Client(client_id=f"gamepad_pub_{socket.gethostname()}")
    mqttc.will_set("rover/gamepad/status",
                   payload=json.dumps({"client":"base_station","status":"offline"}),
                   qos=1, retain=False)

    mqttc.connect(BROKER, PORT, KEEPALIVE)
    mqttc.loop_start()
    mqttc.publish("rover/gamepad/status",
                  json.dumps({"client":"base_station","status":"online"}))

    try:
        while True:
            gp.update()
            drive_msg = gp_to_dict(gp.data)
            mqttc.publish(TOPIC_DRIVE, json.dumps(drive_msg), qos=QOS)
            if getattr(gp, "arm_controller", None):
                arm_msg = gp_to_dict(gp.arm_data)
                mqttc.publish(TOPIC_ARM, json.dumps(arm_msg), qos=QOS)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting publisher")
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()

if __name__ == "__main__":
    main()

