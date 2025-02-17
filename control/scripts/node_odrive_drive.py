# Author: mn297
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from scipy.ndimage import gaussian_filter1d
from msg_srv_interface.msg import WheelSpeed
from msg_srv_interface.msg import MotorState, MotorError, ODriveStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from enum import Enum
from odrive.enums import AxisState, ODriveError, ProcedureResult
from odrive.utils import dump_errors
from ODriveJoint import *
from threading import Lock
from queue import Queue
import threading
import rclpy
from rclpy.node import Node
import scipy.stats as st
from std_msgs.msg import String


class ODrive_node(Node):
    def __init__(self):
        super().__init__("odrive_node")
        self.is_homed = False
        self.is_calibrated = False
        self.threads = []
        self.shutdown_flag = False

        # CONFIGURATION ---------------------------------------------------------------
        # Serial number of the ODrive controlling the joint
        self.joint_serial_numbers = {
            # "rover_drive_rf": "384F34683539",
            "rover_drive_rf": "385C347A3539",
            # "rover_drive_lf": "386134503539",
            "rover_drive_lf": "387134683539",
            # "rover_drive_rb": "387134683539",
            "rover_drive_rb": "386134503539",
            # "rover_drive_lb": "385C347A3539",
            "rover_drive_lb": "384F34683539",
        }

        # VARIABLES -------------------------------------------------------------------
        # Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object
        self.joint_dict = {
            "rover_drive_rf": None,
            "rover_drive_lf": None,
            "rover_drive_rb": None,
            "rover_drive_lb": None,
        }
        self.locks = {joint_name: Lock() for joint_name in self.joint_dict.keys()}

        # Subscriptions
        # Cmd comes from the drive_control_node through the wheel_velocity_cmd topic, then we convert it to setpoint and apply it to the ODrive
        self.drive_cmd_subscriber = self.create_subscription(
            WheelSpeed, "/wheel_velocity_cmd", self.handle_drive_cmd, 1
        )

        # Publishers
        self.drive_fb_publisher = self.create_publisher(
            WheelSpeed, "/wheel_velocity_feedback", 1
        )
        self.odrive_publisher = self.create_publisher(ODriveStatus, "/odrive_state", 1)

        # Frequency of the ODrive I/O
        self.rate = self.create_rate(100)

        # self.run()
        self.publisher_ = self.create_publisher(
            String, "chatter", 10
        )  # Create a publisher

    # Subscriber callback function: Recieve wheelspeed command, and set speed to motor
    def handle_drive_cmd(self, msg):
        msg2 = String()
        msg2.data = f"Hello from handle_drive_cmd: {msg.left[0]}"
        self.publisher_.publish(msg2)
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        try:
            self.joint_dict["rover_drive_lb"].vel_cmd = msg.left[0]
        except KeyError:
            # print("KeyError: 'rover_drive_lb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_lf"].vel_cmd = msg.left[1]
        except KeyError:
            # print("KeyError: 'rover_drive_lf' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rb"].vel_cmd = msg.right[0]
        except KeyError:
            # print("KeyError: 'rover_drive_rb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rf"].vel_cmd = msg.right[1]
        except KeyError:
            # print("KeyError: 'rover_drive_rf' not found in joint_dict")
            pass

        # APPLY Velocity CMD
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            try:
                # setpoint in rev/s
                joint_obj.odrv.axis0.controller.input_vel = (
                    joint_obj.vel_cmd * joint_obj.direction
                )
            except fibre.libfibre.ObjectLostError:
                joint_obj.odrv = None
            except:
                print(
                    f"""Cannot apply vel_cmd {joint_obj.vel_cmd} to joint: {joint_name}"""
                )

    def reconnect_joint(self, joint_name, joint_obj):
        # Attempt to reconnect...
        # Update joint_obj.odrv as necessary...
        joint_obj.reconnect()

        # Once done, reset the flag
        with self.locks[joint_name]:
            joint_obj.is_reconnecting = False

    # Deprecated because enter_closed_loop_control() will call calibrate() if necessary
    def calibrate_and_enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"CALIBRATING joint {joint_obj.name}...")
            joint_obj.calibrate()

            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def publish_joints_feedback(self):
        # PUBLISH Odrive velocity FB
        feedback = WheelSpeed()
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            try:
                self.joint_dict[joint_name].vel_fb = (
                    joint_obj.odrv.encoder_estimator0.vel_estimate
                )
            except fibre.libfibre.ObjectLostError:
                joint_obj.odrv = None
            except:
                print(f"""Cannot get feedback from joint: {joint_name}""")
        # Publish
        try:
            feedback.left[0] = self.joint_dict["rover_drive_lb"].vel_fb
        except KeyError:
            # print("KeyError: 'rover_drive_lb' not found in joint_dict")
            feedback.left[0] = 0.0  # Default value if key is missing
        try:
            feedback.left[1] = self.joint_dict["rover_drive_lf"].vel_fb
        except KeyError:
            # print("KeyError: 'rover_drive_lf' not found in joint_dict")
            feedback.left[1] = 0.0  # Default value if key is missing
        try:
            feedback.right[0] = self.joint_dict["rover_drive_rb"].vel_fb
        except KeyError:
            # print("KeyError: 'rover_drive_rb' not found in joint_dict")
            feedback.right[0] = 0.0  # Default value if key is missing
        try:
            feedback.right[1] = self.joint_dict["rover_drive_rf"].vel_fb
        except KeyError:
            # print("KeyError: 'rover_drive_rf' not found in joint_dict")
            feedback.right[1] = 0.0  # Default value if key is missing
        self.drive_fb_publisher.publish(feedback)

    def setup_odrive(self):
        # CONNECT -----------------------------------------------------
        # for key, value in self.joint_serial_numbers.items():
        for key, value in self.joint_dict.items():
            # Instantiate ODriveJoint class whether or not the connection attempt was made/successful
            # self.joint_dict[key] = ODriveJoint(name=key, serial_number=value)
            self.joint_dict[key] = ODriveJoint(
                name=key, serial_number=self.joint_serial_numbers[key]
            )

            if value == 0:
                print(
                    f"""Skipping connection for joint: {key} due to serial_number being 0"""
                )
                continue

            self.joint_dict[key].reconnect()

        # # CONNECT -----------------------------------------------------
        # for joint_name, joint_obj in self.joint_dict.items():
        #     if joint_obj.odrv is None:
        #         continue
        #     print(f"Creating reconnect() thread for joint {joint_obj.name}")
        #     t = threading.Thread(
        #         target=self.reconnect_joint,
        #         args=(joint_name, joint_obj),
        #     )
        #     self.threads.append(t)
        #     t.start()

        # # Wait for all threads to complete
        # for t in self.threads:
        #     t.join()
        # self.threads = []

        # print("Connection step completed.")

        # CALIBRATE AND ENTER CLOSED LOOP CONTROL -----------------------------------------------------
        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating calibrate() thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.calibrate,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(
                f"Creating enter_closed_loop_control() thread for joint {joint_obj.name}"
            )
            t = threading.Thread(
                target=joint_obj.enter_closed_loop_control,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        self.is_calibrated = True

        # Set the direction of the motors
        try:
            self.joint_dict["rover_drive_lb"].direction = -1
        except KeyError:
            # print("KeyError: 'rover_drive_lb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_lf"].direction = -1
        except KeyError:
            # print("KeyError: 'rover_drive_lf' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rb"].direction = 1
        except KeyError:
            # print("KeyError: 'rover_drive_rb' not found in joint_dict")
            pass
        try:
            self.joint_dict["rover_drive_rf"].direction = 1
        except KeyError:
            # print("KeyError: 'rover_drive_rf' not found in joint_dict")
            pass
        print("Entering closed loop control step completed.")

    # SEND ODRIVE INFO AND HANDLE ERRORS
    def handle_joints_error(self):
        for joint_name, joint_obj in self.joint_dict.items():
            with self.locks[joint_name]:  # Use a lock specific to each joint
                if joint_obj.odrv is not None:
                    if (
                        joint_obj.odrv.axis0.current_state
                        != AxisState.CLOSED_LOOP_CONTROL
                    ):
                        if not joint_obj.is_reconnecting:
                            print(
                                f"{joint_name} is not in closed loop control, recalibrating..."
                            )
                            joint_obj.is_reconnecting = True
                            t = threading.Thread(
                                target=self.enter_closed_loop_control,
                                # target=self.calibrate_and_enter_closed_loop,
                                args=(joint_obj,),
                                # target=joint_obj.enter_closed_loop_control
                            )
                            self.threads.append(t)
                            t.start()
                    else:
                        # Handle case where 'axis0' is not available
                        # print(
                        #     f"Cannot check current state for {joint_name}, 'axis0' attribute missing."
                        # )
                        pass
                else:
                    # ODrive interface is None, indicating a disconnection or uninitialized state
                    if not joint_obj.is_reconnecting:
                        print(f"RECONNECTING {joint_name}...")
                        joint_obj.is_reconnecting = True
                        t = threading.Thread(
                            target=self.reconnect_joint,
                            args=(joint_name, joint_obj),
                        )
                        t.start()

    def print_loop(self):
        while not self.shutdown_flag:
            print_joint_state_from_dict(self.joint_dict, sync_print=True)
            time.sleep(0.2)

    def loop_odrive(self):
        # MAIN LOOP -----------------------------------------------------
        while rclpy.ok() and not self.shutdown_flag:
            self.publish_joints_feedback()

            # HANDLE ERRORS
            self.handle_joints_error()

            # Wait at rate freq
            self.rate.sleep()

    def run(self):
        self.setup_odrive()
        thread = threading.Thread(target=self.loop_odrive)
        thread.start()
        print_thread = threading.Thread(target=self.print_loop)
        print_thread.start()
        # rclpy.spin()
        self.shutdown_hook()

    def shutdown_hook(self):
        self.shutdown_flag = True
        print("Shutdown initiated. Setting all motors to idle state.")
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE
        for t in self.threads:
            t.join()
        print("All threads joined. Shutdown complete.")


# ROS runtime main entry point
def main():
    rclpy.init()
    odrive_node = ODrive_node()
    odrive_node.run()  # Start the threads after the node is created
    rclpy.spin(odrive_node)
    odrive_node.shutdown_hook()
    odrive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
