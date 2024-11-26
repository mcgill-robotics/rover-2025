import rospy
# TODO: figure out correct pathway
from human_control_interface.msg import Gamepad_input
from camera_data.msg import Camera_Orientation
from geometry_msgs.msg import Twist
from angular_gamepad import AngularGamepad
from std_msgs.msg import Float32MultiArray

class Node_GamepadProcessing:
    def __init__(self, v_max, w_max):
        """
        the member variables of the GamepadProcess object

        gamepad: Reference to a gamepad object
        roverLinearVelocity: the rover's linear velocity
        maxLinearVelocity: the upper limit set for the linear velocity
        maxAngularVelocity: the lower limit set for the angular velocity

        NOTE: twist measurements are in SI units (m/s, m, ...)
        """
        # init ROS node
        rospy.init_node("gamepad_process_node")

        # init gamepad object
        self.gamepad_init_successful = False
        try:
            self.gamepad = AngularGamepad()
            self.gamepad_init_successful = True
        except:
            print("Controller not found. Falling back to debug control")
            self.listener = keyboard.Listener(on_press=self.keyboardProcessCall)
            self.keyboard_accumulator_linear = 0.0
            self.keyboard_accumulator_twist = 0.0
            self.keyboard_sensitivity = 0.05
            self.listener.start()

        # init variables for velocity
        self.roverLinearVelocity = 0
        self.roverAngularVelocity = 0

        # init variables for rover max linear and angular velocity
        self.maxLinearVelocity = v_max
        self.maxAngularVelocity = w_max

        # TODO: get rid of camera import stuff? talk to a lead
        # init variables for camera
        self.cam_ctrl = Float32MultiArray()
        self.cam_ctrl.data = [0, 0] # elements: [x-axis, y-axis]

        # publisher for twist values
        self.drive_publisher = rospy.Publisher("rover_velocity_controller/cmd_vel", Twist, queue_size=1)
        # publisher for pan tilt camera angles
        self.camera_publisher = rospy.Publisher("panTiltAngles", Float32MultiArray, queue_size=1)

        # control frequency of node
        self.rate = rospy.Rate(100)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                exit()
            try:

                # skip the rest of the loop if gamepad is not connected
                if not self.gamepad_init_successful:
                    continue

                self.gamepad.update()
                msg = Gamepad_input()

                #transfer data into msg
                msg.B1 = self.gamepad.data.b1
                msg.B2 = self.gamepad.data.b2
                msg.B3 = self.gamepad.data.b3
                msg.B4 = self.gamepad.data.b4
                msg.B5 = self.gamepad.data.b5
                msg.B6 = self.gamepad.data.b6
                msg.B7 = self.gamepad.data.b7
                msg.B8 = self.gamepad.data.b8
                msg.B9 = self.gamepad.data.b9
                msg.B10 = self.gamepad.data.b10
                msg.B11 = self.gamepad.data.b11
                msg.B12 = self.gamepad.data.b12
                msg.B13 = self.gamepad.data.b13
                msg.A1 = self.gamepad.data.a1
                msg.A2 = self.gamepad.data.a2
                msg.A3 = self.gamepad.data.a3
                msg.A4 = self.gamepad.data.a4
                msg.A5 = self.gamepad.data.a5
                msg.A6 = self.gamepad.data.a6
                # passes msg (gamepad data) to gamepadProcessCall
                self.gamepadProcessCall(msg)

            except Exception as error:
                rospy.logerr(str(error))

            self.rate.sleep()

        exit()

    def keyboardProcessCall(self, key):
        if key == keyboard.Key.up:
            self.keyboard_accumulator_linear += self.keyboard_sensitivity
        if key == keyboard.Key.down:
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
            self.keyboard_accumulator_twist = -1.0

        self.roverLinearVelocity = self.maxLinearVelocity * self.keyboard_accumulator_linear
        self.roverAngularVelocity = self.maxAngularVelocity * self.keyboard_accumulator_twist

        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.drive_publisher.publish(roverTwist)

    # poll gamepad data and call the respective process call
    def gamepadProcessCall(self, msg):
        self.driveProcessCall(msg)
        self.cameraProcessCall(msg)

    def driveProcessCall(self, msg):
        # a2 is moving the left stick up and down- drives forwards or backwards
        # a4 is the right stick thar moves left and right- steers left or right

        if abs(msg.A4) < 0.1:
            msg.A4 = 0
        if abs(msg.A2) < 0.1:
            msg.A2 = 0

        drive = msg.A2
        steer = msg.A4

        # calc linear velocity
        self.roverLinearVelocity = self.maxLinearVelocity * drive * drive * drive
        #calc for angular velocity
        self.roverAngularVelocity = self.maxAngularVelocity * steer * steer * steer

        # assign values to a twist msg, then publish to ROS
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.drive_publisher.publish(roverTwist)

    def cameraProcessCall(self, msg):
        '''
        up: triangle
        down: X
        right: O
        left: square
        '''

        if (msg.B3 == 1 or msg.B1 == 1) and (self.cam_ctrl.data[0] + msg.B3 <= 180) and (self.cam_ctrl.data[0] - msg.B1 >=0):
            self.cam_ctrl.data[0] = self.cam_ctrl.data[0] + msg.B3 - msg.B1
            print(f"horizontal: {self.cam_ctrl.data[1]}")

        self.camera_publisher.publish(self.cam_ctrl)

    # TODO: ask someone what this does v
    def risingEdge(self, prevSignal, nextSignal):
        if prevSignal < nextSignal:
            return True
        else:
            return False

if __name__ == "__main__":
    gamepadProcess = Node_GamepadProcessing(5, 10)