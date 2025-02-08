#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from pynput import keyboard
from msg_interface.msg import GamePadInput
from sensor_msgs.msg import Joy
#from camera_data.msg import Camera_Orientation
from geometry_msgs.msg import Twist
# from Gamepad import Gamepad
from Gamepad import Gamepad
# from Gamepad import *
from std_msgs.msg import Float32MultiArray



class Node_GamepadProcessing(Node):
    def __init__(self):
        """
        The member variables of the GamepadProcess object

        gamepad: Reference to a Gamepad object.

        roverLinearVelocity: The rover's linear velocity
        roverAngularVelocity: The rover's angular velocity
        maxLinearVelocity: The upper limit set for linear velocity.
        maxAngularVelocity: The lower limit set for angular velocity

        NOTE: Twist measurements are in SI units. (m/s, m, ...)

        """
        # initialize ROS node
        super().__init__("gamepad_process_node")

        # Initialize a Gamepad object
        self.gamepad_init_successful = False
        try:
            self.gamepad = Gamepad()
            self.gamepad_init_successful = True
        except:
            print("Controller not found. Falling back to debug keyboard control")
            self.listener = keyboard.Listener(on_press=self.keyboardProcessCall)
            self.keyboard_accumulator_linear = 0.0
            self.keyboard_accumulator_twist = 0.0
            self.keyboard_sensitivity = 0.05
            self.listener.start()


        #Declare publisher for Joystick msg
        self.gamepad_publisher = self.create_publisher(Joy, "gamepad_input", 10)

        # Control frequency of the node
        self.rate = self.create_rate(100)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    # The run loop that updates a controller's value.
    def run(self):
        while rclpy.ok():
            try:

                # Skip the rest of the loop if gamepad is not connected
                if not self.gamepad_init_successful:
                    continue

                self.gamepad.update()
                msg = GamePadInput()

                # Transfer Data into msg
                b_1 = self.gamepad.data.b1
                b_2 = self.gamepad.data.b2
                b_3 = self.gamepad.data.b3
                b_4 = self.gamepad.data.b4
                b_5 = self.gamepad.data.b5
                b_6 = self.gamepad.data.b6
                b_7 = self.gamepad.data.b7
                b_8 = self.gamepad.data.b8
                b_9 = self.gamepad.data.b9
                b_10 = self.gamepad.data.b10
                b_11 = self.gamepad.data.b11
                b_12 = self.gamepad.data.b12
                b_13 = self.gamepad.data.b13
                a_1 = self.gamepad.data.a1
                a_2 = self.gamepad.data.a2
                a_3 = self.gamepad.data.a3
                a_4 = self.gamepad.data.a4
                a_5 = self.gamepad.data.a5
                a_6 = self.gamepad.data.a6
                d_1 = float(self.gamepad.data.a7[0])
                d_2 = float(self.gamepad.data.a7[1])


                
                ctrl_inp = Joy()
                ctrl_inp.axes = [a_1, 
                                 a_2, 
                                 a_3, 
                                 a_4,
                                 a_5,
                                 a_6,
                                 d_1,
                                 d_2]
                
                ctrl_inp.buttons = [b_1,
                                    b_2,
                                    b_3,
                                    b_4,
                                    b_5,
                                    b_6,
                                    b_7,
                                    b_8,
                                    b_9,
                                    b_10,
                                    b_11,
                                    b_12,
                                    b_13]
                
                self.gamepad_publisher.publish(ctrl_inp)
            except Exception as error:
                    print
                    self.get_logger().info(str(error))





def main(args=None):
    rclpy.init(args=args)
    driver = Node_GamepadProcessing()
    rclpy.spin(driver)

if __name__ == "__main__":
    main()

