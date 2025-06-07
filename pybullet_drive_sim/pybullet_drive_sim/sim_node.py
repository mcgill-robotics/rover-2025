import os
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import time

from ament_index_python.packages import get_package_share_directory


class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')

        # Start PyBullet GUI
        p.connect(p.GUI)

        # Add default PyBullet search path
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load plane
        p.loadURDF("plane.urdf")

        # Find path to your URDF inside your ROS 2 package
        package_path = get_package_share_directory('pybullet_drive_sim')
        urdf_path = os.path.join(package_path, 'urdf', 'rover.urdf')

        # Load your actual rover URDF
        p.loadURDF(urdf_path, [0, 0, 1])

        self.get_logger().info(f"Loaded URDF: {urdf_path}")

def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    try:
        while rclpy.ok():
            p.stepSimulation()
            time.sleep(1. / 240.)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
