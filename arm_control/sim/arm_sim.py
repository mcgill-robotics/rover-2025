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

sys.path.append("..")


class Node_ArmSim(Node):

    def __init__(self):
        p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.1])

        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../model/MR_arm.urdf"
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1)

        p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], [0, 0, 0, 1])
        self.numJoints = p.getNumJoints(self.robotId)
        self.endEffectorIndex = self.numJoints - 1

        for i in range(self.numJoints):
            p.resetJointState(self.robotId, i, 0)

        p.setGravity(0, 0, -9.8)
        self.prevPose = [0, 0, 0]
        self.hasPrevPose = 0
        self.t = 0.0

        p.setRealTimeSimulation(0)
        self.trailDuration = 5

        self.jointPoses = [0] * self.numJoints
        self.jointVels = [0] * self.numJoints
        self.jointTorq = [0] * self.numJoints

        self.desiredJointPos = [0] * self.numJoints

        node = rclpy.create_node("arm_sim", anonymous=False)

        self.armBrushedSubscriber = node.create_subscription(
            "armBrushedCmd", Float32MultiArray, self.updateArmBrushedSim
        )
        self.armBrushlessSubscriber = node.create_subscription(
            "armBrushlessCmd", Float32MultiArray, self.updateArmBrushlessSim
        )
        self.armBrushedPublisher = node.create_publisher(
            "armBrushedFb", Float32MultiArray, queue_size=10
        )
        self.armBrushlessPublisher = node.create_publisher(
            "armBrushlessFb", Float32MultiArray, queue_size=10
        )

        self.run()

    def updateArmBrushedSim(self, cmds: Float32MultiArray):
        self.desiredJointPos[4], self.desiredJointPos[3] = tuple(
            x * (pi / 180) for x in cmds.data[1:]
        )
        self.desiredJointPos[5] = np.clip(
            self.desiredJointPos[5] + cmds.data[0], -0.3, 0.11
        )
        self.desiredJointPos[6] = self.desiredJointPos[5]
        print("Updated desiredJointPos for brushed motors:", self.desiredJointPos)

    def updateArmBrushlessSim(self, cmds: Float32MultiArray):
        self.desiredJointPos[2], self.desiredJointPos[1], self.desiredJointPos[0] = (
            tuple(x * (pi / 180) for x in cmds.data)
        )
        print("Updated desiredJointPos for brushless motors:", self.desiredJointPos)

    def run(self):
        while not rclpy.ok():
            self.t = self.t + 0.01
            p.stepSimulation()

            for i in range(self.numJoints):

                p.setJointMotorControl2(
                    bodyIndex=self.robotId,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=self.desiredJointPos[i],
                    force=500,
                    positionGain=0.03,
                    velocityGain=1,
                )

            ls = p.getLinkState(self.robotId, self.endEffectorIndex)

            if self.hasPrevPose:
                p.addUserDebugLine(
                    self.prevPose, ls[4], [1, 0, 0], 1, self.trailDuration
                )
            self.prevPose = ls[4]
            self.hasPrevPose = 1

            states = p.getJointStates(self.robotId, range(self.numJoints))
            for i in range(len(states)):
                self.jointPoses[i] = states[i][0] * (180 / pi)
                self.jointVels[i] = states[i][1]
                self.jointTorq[i] = states[i][3]

            state_brushed_msg = Float32MultiArray()
            state_brushed_msg.data = (
                self.jointPoses[5],
                self.jointPoses[4],
                self.jointPoses[3],
            )

            state_brushless_msg = Float32MultiArray()
            state_brushless_msg.data = (
                self.jointPoses[2],
                self.jointPoses[1],
                self.jointPoses[0],
            )

            self.armBrushedPublisher.publish(state_brushed_msg)
            self.armBrushlessPublisher.publish(state_brushless_msg)

        p.disconnect()


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    driver = Node_ArmSim()
    rclpy.spin(driver)
