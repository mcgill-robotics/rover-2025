import socket
import pygame
import time


JETSON_IP = "192.168.0.101"  # IP of the motor computer
UDP_PORT = 5005           # Port to send data


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP socket


def connect_joystick():
    # Initialize pygame for joystick input
    pygame.init()
    pygame.joystick.init()

    # Attempt to open the first joystick; wait until the user connects the PS4 controller
    joystick = None
    while joystick is None:
        count = pygame.joystick.get_count()
        if count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Detected joystick: {joystick.get_name()}")
        else:
            print("No joystick detected. Please connect a PS4 controller.")
            time.sleep(2)
            pygame.joystick.quit()
            pygame.joystick.init()

    print("Press Ctrl+C to quit.\n")

    return joystick

joystick = connect_joystick()

while True:
    
    # Pump the pygame event loop so we can read joystick values
    pygame.event.pump()

    # Typically, axis(0) is the left stick X axis, axis(1) is the left stick Y axis
    x_axis = joystick.get_axis(0)  # range -1.0 ... +1.0
    y_axis = joystick.get_axis(1)  # range -1.0 ... +1.0

    # Example usage:
    # Let the left stick Y-axis control motor speed:
    #   -1.0 => maximum forward
    #    1.0 => maximum backward
    # We can scale that range to your desired speed range, e.g. [-1000, 1000].
    max_speed = 1500.0
    speed_command = -y_axis * max_speed  # invert sign if you prefer stick up = positive speed

    # Decide direction (FORWARD_CW or BACKWARD_CCW) based on sign
    if speed_command >= 0:
        direction = 0x00
    else:
        direction = 0x01

    # Because your ESC code expects "run_speed(value, direction)" in absolute terms:
    abs_speed_val = abs(speed_command)

    # Send the speed command
    print(abs_speed_val, direction)
    command = str(abs_speed_val) + ":" + str(direction)
    #command = input("Enter command: ")  # Example: "MOVE_FORWARD"
    sock.sendto(command.encode(), (JETSON_IP, UDP_PORT))

    # Adjust this delay as needed
    time.sleep(0.25)