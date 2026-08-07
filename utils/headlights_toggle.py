# To toggle the headlights on, type in the terminal:  python headlights_toggle.py on 
# To toggle the headlights off, type in the terminal: python headlights_toggle.py off
# Make sure you modify your COM port properly 

import serial
import time
import sys

PORT = "COM6"  # Change this to match the right port
BAUDRATE = 9600

# Argument check
if len(sys.argv) != 2:
    print("Usage: python killswitch.py [on|off]")
    sys.exit(1)

cmd = sys.argv[1].lower()
if cmd not in ["on", "off"]:
    print("Invalid argument. Use 'on' or 'off'.")
    sys.exit(1)

cmd_map = {
    "on": b'H',
    "off": b'L',
}

try:
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)  # Wait for Teensy to initialize
        ser.write(cmd_map[cmd])
        print(f"Sent command '{cmd}' to Teensy on {PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")