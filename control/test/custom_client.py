import rclpy
from rclpy.node import Node
from msg_srv_interface import DriveMotorStatus

"""
Test for the Motor Status Ping ROS Service on drive_firmware_node.py
Waits for the ROS Service to get information about the current state of the motors.
"""

def main():
    rclpy.init()
    node = Node("test-client")
    client = node.create_client(DriveMotorStatus, "drive_motor_status")
    while not client.wait_for_service(timeout_sec=1.0):
        print("waiting for service")
    req = DriveMotorStatus.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        print("RF_OK:", future.result().RF_OK)
        print("RB_OK:", future.result().RB_OK)
        print("LB_OK:", future.result().LB_OK)
        print("LF_OK:", future.result().LF_OK)
    node.destroy_node()
    rclpy.shutdown()