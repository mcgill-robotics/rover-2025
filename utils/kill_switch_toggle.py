# To kill KS2, write in terminal:   python kill_switch_toggle.py on
# To unkill KS2, write in terminal: python kill_switch_toggle.py off
# Make sure you modify your COM port properly 

import serial
import time
import sys

PORT = "/dev/ttyACM2"  # Change this to the correct port
BAUDRATE = 9600

# Argument check
if len(sys.argv) != 2:
    print("Usage: python killswitch.py [on|off]")
    sys.exit(1)

cmd = sys.argv[1].lower()
if cmd not in ["on", "off"]:
    print("Invalid argument. Use 'on' or 'off'.")
    sys.exit(1)

# Map command to byte
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
