import os
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
import time, math

class drive_simulation_node(Node):
    def __init__(self):
        super().__init__('drive_simulation_node')

        # Start PyBullet GUI
        p.connect(p.GUI)

        # Add default PyBullet search path
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load world
        p.loadURDF("plane.urdf", [0, 0, -0.1])
        p.setGravity(0, 0, -9.81)

        # Load your actual rover URDF
        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/ROVER_DRIVE.urdf"
        self.roverID = p.loadURDF(urdf_path, [0, 0, 1], useFixedBase=False)
        self.get_logger().info(f"Loaded URDF: {urdf_path}")

        self.t = 0.0
        p.setRealTimeSimulation(0)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.run)

    def run(self):
        self.t = self.t + 0.01
        p.stepSimulation()

def main(args=None):
    rclpy.init(args=args)
    drive_sim_node = drive_simulation_node()
    try:
        rclpy.spin(drive_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        drive_sim_node.destroy_node()
        rclpy.shutdown()
        p.disconnect()

if __name__ == "__main__":
    main()