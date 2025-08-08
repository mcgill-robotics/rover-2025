import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

def main():
    rclpy.init()
    node = Node("test_client")
    client = node.create_client(Trigger, "calibration_service")
    while not client.wait_for_service(timeout_sec=1.0):
        print("waiting for service")
    req = Trigger.Request()
    print('hello')
    future = client.call(req)
    print('hellooo')
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        print(future.result().success)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
