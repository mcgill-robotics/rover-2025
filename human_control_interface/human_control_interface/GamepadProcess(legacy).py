#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from pynput import keyboard
from msg_interface.msg import GamePadInput
#from camera_data.msg import Camera_Orientation
from geometry_msgs.msg import Twist
# from Gamepad import Gamepad
from Gamepad import Gamepad
# from Gamepad import *
from std_msgs.msg import Float32MultiArray



class Node_GamepadProcessing(Node):
    def __init__(self, v_max, w_max):
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
            #self.listener = keyboard.Listener(on_press=self.keyboardProcessCall)
            self.keyboard_accumulator_linear = 0.0
            self.keyboard_accumulator_twist = 0.0
            self.keyboard_sensitivity = 0.05
            self.listener.start()


        # initialize variables for velocity
        self.roverLinearVelocity = 0
        self.roverAngularVelocity = 0

        # initialize variables for rover's max linear and angular velocities
        self.maxLinearVelocity = v_max
        self.maxAngularVelocity = w_max

        # Initialize variables for camera
        self.cam_ctrl = Float32MultiArray()
        self.cam_ctrl.data = [0.0, 0.0] # Elements: [X-axis, Y-axis]

        #self.drive_publisher = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=1) # Publisher for twist values.
        self.drive_publisher = self.create_publisher(Twist, "rover_velocity_controller/cmd_vel", 10)
        #self.camera_publisher = rospy.Publisher("panTiltAngles", Float32MultiArray, queue_size=1) # Publisher for pan tilt camera angles.
        self.camera_publisher = self.create_publisher(Float32MultiArray, "panTiltAngles",10)

        # Control frequency of the node
        self.rate = self.create_rate(100)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    # The run loop that updates a controller's value.
    def run(self):
        while rclpy.ok():
            # if not rclpy.ok():
            #     exit()
            try:

                # Skip the rest of the loop if gamepad is not connected
                if not self.gamepad_init_successful:
                    continue

                self.gamepad.update()
                msg = GamePadInput()

                # Transfer Data into msg
                msg.b_1 = self.gamepad.data.b1
                msg.b_2 = self.gamepad.data.b2
                msg.b_3 = self.gamepad.data.b3
                msg.b_4 = self.gamepad.data.b4
                msg.b_5 = self.gamepad.data.b5
                msg.b_6 = self.gamepad.data.b6
                msg.b_7 = self.gamepad.data.b7
                msg.b_8 = self.gamepad.data.b8
                msg.b_9 = self.gamepad.data.b9
                msg.b_10 = self.gamepad.data.b10
                msg.b_11 = self.gamepad.data.b11
                msg.b_12 = self.gamepad.data.b12
                msg.b_13 = self.gamepad.data.b13
                msg.a_1 = self.gamepad.data.a1
                msg.a_2 = self.gamepad.data.a2
                msg.a_3 = self.gamepad.data.a3
                msg.a_4 = self.gamepad.data.a4
                msg.a_5 = self.gamepad.data.a5
                msg.a_6 = self.gamepad.data.a6
                
                msg.d_1 = float(self.gamepad.data.a7[0])
                msg.d_2 = float(self.gamepad.data.a7[1])

                print(msg.d_1)
                print(msg.d_2)
                #Passes msg (gamepad data) to gamepadProcessCall
                self.gamepadProcessCall(msg)
            except Exception as error:
                    print
                    self.get_logger().info(str(error))

            #self.rate.sleep()

        #exit()

    def keyboardProcessCall(self, key):
        if key == keyboard.Key.up:
            # self.roverLinearVelocity = 10
            self.keyboard_accumulator_linear += self.keyboard_sensitivity
        if key == keyboard.Key.down:
            # self.roverLinearVelocity = -10
            self.keyboard_accumulator_linear -= self.keyboard_sensitivity
        if key == keyboard.Key.left:
            self.keyboard_accumulator_twist -= self.keyboard_sensitivity
        if key == keyboard.Key.right:
            self.keyboard_accumulator_twist += self.keyboard_sensitivity
        if key == keyboard.KeyCode.from_char('0'):
            self.keyboard_accumulator_linear = 0.0
            self.keyboard_accumulator_twist = 0.0

        if self.keyboard_accumulator_linear > 1.0:
            self.keyboard_accumulator_linear = 1.0
        elif self.keyboard_accumulator_linear < -1.0:
            self.keyboard_accumulator_linear = -1.0

        if self.keyboard_accumulator_twist > 1.0:
            self.keyboard_accumulator_twist = 1.0
        elif self.keyboard_accumulator_twist < -1.0:
            self.keyboard_accumulator_twist= -1.0
        self.roverLinearVelocity = self.maxLinearVelocity * self.keyboard_accumulator_linear
        self.roverAngularVelocity = self.maxAngularVelocity * self.keyboard_accumulator_twist

        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        #time.sleep(0.5)
        self.drive_publisher.publish(roverTwist)

    # Poll the gamepad data and then call the respective process call.
    def gamepadProcessCall(self, msg):
        self.driveProcessCall(msg)
        #self.cameraProcessCall(msg)

    def driveProcessCall(self, msg):
        # A2 is the left stick moving up and down. Drives forwards or backwards.
        # A4 is the right stick that moves left and right. Steers left or right.

        if abs(msg.a_4) < 0.1:
            msg.a_4 = 0.0

        print("in driiverProcessCall")
        if abs(msg.a_2) < 0.1:
            msg.a_2 = 0.0

        drive = msg.a_2
        steer = msg.a_4

        # calc. for linear velocity
        self.roverLinearVelocity = self.maxLinearVelocity * drive * drive * drive

        # calc. for angular velocity
        self.roverAngularVelocity = self.maxAngularVelocity * steer * steer * steer

        # Assigns values to a Twist msg, then publish it to ROS
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.drive_publisher.publish(roverTwist)


    def cameraProcessCall(self, msg):
        # up: triangle
        # down: X
        # right: O
        # left: square

        if (msg.b_3 == 1 or msg.b_1 == 1) and (self.cam_ctrl.data[0] + msg.b_3 <= 180) and (self.cam_ctrl.data[0] - msg.b_1 >= 0):
            self.cam_ctrl.data[0] = self.cam_ctrl.data[0] + msg.b_3 - msg.b_1
            print(f"vertical: {self.cam_ctrl.data[0]}")

        if (msg.b_4 == 1 or msg.b_2 == 1) and (self.cam_ctrl.data[1] + msg.b_2 <= 180) and (self.cam_ctrl.data[1] - msg.b_4 >= 0):
            self.cam_ctrl.data[1] = self.cam_ctrl.data[1] + msg.b_2 - msg.b_4
            print(f"horizontal: {self.cam_ctrl.data[1]}")

        self.camera_publisher.publish(self.cam_ctrl)


    def risingEdge(self, prevSignal, nextSignal):
        if prevSignal < nextSignal:
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    driver = Node_GamepadProcessing(5, 10)
    #driver.gamepad.printData()
    rclpy.spin(driver)

if __name__ == "__main__":
    main()
    #rospy.spin()
