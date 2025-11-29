# Rover Telemetry Communications R&D

Daniel Wei (Rover Software Subteam)

## The MQTT Transmission Protocol

### Technology Framework Breakdown

1. JSON File format for transmission protocol information

- sample file format in `gamepad_input.json`
- modified `gamepad_input_mqtt_pub.py` for handling the communication script done with ROS
- new `gamepad_Mqtt_sub.py` file to handle the requests (in the works)

2. Broker setup using Docker (for packaging to both Windows and MAC OS)

```bash
# pull and run Mosquitto (ports 1883 for MQTT, 9001 for websockets optional)
docker run -d --name mosquitto \
  -p 1883:1883 -p 9001:9001 \
  Eclipse-mosquitto:2.0
```

3. Verify Broker Setup  
   Test with built-in Mosquitto tools:

```bash
# Terminal 1 (subscriber)

mosquitto_sub -h localhost -t 'rover/#' -v

# Terminal 2 (publisher)

mosquitto_pub -h localhost -t 'rover/gamepad/drive' -m '{"test":1}'
```

4. Python environment Setup

```bash
   #On both Jetson and Base Station:
   python3 -m venv venv
   source venv/bin/activate
   pip install --upgrade pip
   pip install paho-mqtt
```

5. Testing Procedure

- Start the Mosquitto broker:

```bash

docker start mosquitto

#On the Jetson:
mosquitto_sub -h localhost -t 'rover/#' -v

#On the base station (controls):
python3 gamepad_input_mqtt_pub.py
```

### Notes

- Current code in `gamepad_input_mqtt_pub.py` contains the "filler" code for the robot drive, must replace with actual drive code during implementation
- Additional Documentation can be found on the [official mqtt website](https://mqtt.org/)
- This document is a work in progress, stay tuned for more issues!
