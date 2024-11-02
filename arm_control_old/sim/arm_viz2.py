#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from numpy import pi
import rclpy 
from rclpy.node import Node
import numpy as np
import pybullet as p
import pybullet_data
import sys
import os
from threading import Thread

sys.path.append("..")


class Node_ArmVisualizer(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__("arm_visualizer")

        # Initialize PyBullet and load the environment
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.1])

        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/MR_arm.urdf"

        # Load the robot URDF twice
        self.robotIdCmd = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)
        self.robotIdFb = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)

        self.numJoints = p.getNumJoints(self.robotIdCmd)
        self.endEffectorIndex = self.numJoints - 1

        # Change the color of the commanded robot to green
        for i in range(self.numJoints):
            p.changeVisualShape(self.robotIdCmd, i, rgbaColor=[0, 1, 0, 1])

        # Change the color of the feedback robot to red
        for i in range(self.numJoints):
            p.changeVisualShape(self.robotIdFb, i, rgbaColor=[1, 0, 0, 1])

        for i in range(self.numJoints):
            p.resetJointState(self.robotIdCmd, i, 0)
            p.resetJointState(self.robotIdFb, i, 0)

        # Disable collisions between all links of the commanded and feedback robots
        for j in range(self.numJoints):
            p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, -1, j, 0)
            p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, j, -1, 0)
            for k in range(self.numJoints):
                p.setCollisionFilterPair(self.robotIdCmd, self.robotIdFb, j, k, 0)

        # Reset the commanded and feedback robot joint states
        for i in range(self.numJoints):
            p.resetJointState(self.robotIdCmd, i, 0)
            p.resetJointState(self.robotIdFb, i, 0)

        # p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)

        self.jointPosesCmd = [0] * self.numJoints
        self.jointPosesFb = [0] * self.numJoints

        # Subscribers for command and feedback positions
        self.armBrushedCmdSubscriber = self.create_subscription(
            Float32MultiArray, "armBrushedCmd", self.updateArmBrushedCmd, 10
        )
        self.armBrushlessCmdSubscriber = self.create_subscription(
            Float32MultiArray, "armBrushlessCmd", self.updateArmBrushlessCmd, 10
        )
        self.armBrushedFbSubscriber = self.create_subscription(
            Float32MultiArray, "armBrushedFb", self.updateArmBrushedFb, 10
        )
        self.armBrushlessFbSubscriber = self.create_subscription(
            Float32MultiArray, "armBrushlessFb", self.updateArmBrushlessFb, 10
        )

        self.run()

    def updateArmBrushedCmd(self, data: Float32MultiArray):
        print("Received brushed command data:", data.data)
        self.jointPosesCmd[4], self.jointPosesCmd[3] = tuple(
            x * (pi / 180) for x in data.data[1:]
        )
        self.jointPosesCmd[5] = np.clip(
            self.jointPosesCmd[5] + data.data[0], -0.3, 0.11
        )
        self.jointPosesCmd[6] = self.jointPosesCmd[5]
        self.applyJointPositions()
        print("Updated jointPosesCmd for brushed motors:", self.jointPosesCmd)

    def updateArmBrushlessCmd(self, data: Float32MultiArray):
        print("Received brushless command data:", data.data)
        self.jointPosesCmd[2], self.jointPosesCmd[1], self.jointPosesCmd[0] = tuple(
            x * (pi / 180) for x in data.data
        )
        self.applyJointPositions()
        print("Updated jointPosesCmd for brushless motors:", self.jointPosesCmd)

    def updateArmBrushedFb(self, data: Float32MultiArray):
        print("Received brushed feedback data:", data.data)
        self.jointPosesFb[4], self.jointPosesFb[3] = tuple(
            x * (pi / 180) for x in data.data[1:]
        )
        self.jointPosesFb[5] = data.data[0] * (pi / 180)
        self.jointPosesFb[6] = self.jointPosesFb[5]
        self.applyFeedbackJointPositions()
        print("Updated jointPosesFb for brushed motors:", self.jointPosesFb)

    def updateArmBrushlessFb(self, data: Float32MultiArray):
        print("Received brushless feedback data:", data.data)
        self.jointPosesFb[2], self.jointPosesFb[1], self.jointPosesFb[0] = tuple(
            x * (pi / 180) for x in data.data
        )
        self.applyFeedbackJointPositions()
        print("Updated jointPosesFb for brushless motors:", self.jointPosesFb)

    def applyJointPositions(self):
        for i in range(self.numJoints):
            p.resetJointState(
                bodyUniqueId=self.robotIdCmd,
                jointIndex=i,
                targetValue=self.jointPosesCmd[i],
            )

    def applyFeedbackJointPositions(self):
        for i in range(self.numJoints):
            p.resetJointState(
                bodyUniqueId=self.robotIdFb,
                jointIndex=i,
                targetValue=self.jointPosesFb[i],
            )

    def run(self):
        rate = self.create_rate(50)  # 50 Hz 
        while rclpy.ok():
            p.stepSimulation()
            # rate.sleep()

        p.disconnect()


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    visualizer = Node_ArmVisualizer()
    spinner = Thread(target=rclpy.spin, args=(visualizer,))
    spinner.start()
    visualizer.run()
    spinner.join()
    visualizer.destroy_node()
    rclpy.shutdown()
    p.disconnect()
# if __name__ == "__main__":
#     visualizer = Node_ArmVisualizer()
#     rospy.spin()
