import rclpy
from rclpy.node import Node
from msg_srv_interface.srv import DriveMotorStatus

"""
Context: 
* We want to be able ping to motor status to see what motors are available/connected peroidically through the UI
* Test for the Motor Status Ping ROS Service on drive_firmware_node.py
* Waits for the ROS Service to get information about the current state of the motors.

To run:
1st Terminal -> python3 drive_firmware_node.py # in the drive/scripts directory
2nd Terminal -> python3 motor_status_test.py   # in the drive/test directory
"""

def main():
    rclpy.init()
    node = Node("test_client")
    client = node.create_client(DriveMotorStatus, "drive_motors_status")
    while not client.wait_for_service(timeout_sec=1.0):
        print("waiting for service")
    req = DriveMotorStatus.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        print("RF_OK:", future.result().rf_ok)
        print("RB_OK:", future.result().rb_ok)
        print("LB_OK:", future.result().lb_ok)
        print("LF_OK:", future.result().lf_ok)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()