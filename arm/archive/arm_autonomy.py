#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
import numpy as np
from typing import Set, Tuple, List
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from queue import PriorityQueue
import message_filters
from time import sleep
from sklearn.cluster import DBSCAN
import cv2


class ArmMoving:
    shoulder_elbow = 0.5
    elbow_claw = 0.4

    def __init__(self) -> None:
        rospy.init_node("publisher_and_listener", anonymous=True)

    def pubs_and_subs(self):
        self.tumor_pub = rospy.Publisher(
            "/joint_1_controller/command", Float64, queue_size=10
        )
        self.shoulder_pub = rospy.Publisher(
            "/joint_2_controller/command", Float64, queue_size=10
        )
        self.elbow_pub = rospy.Publisher(
            "/joint_3_controller/command", Float64, queue_size=10
        )
        self.wristpitch_pub = rospy.Publisher(
            "/joint_4_controller/command", Float64, queue_size=10
        )
        self.wristroll_pub = rospy.Publisher(
            "/joint_5_controller/command", Float64, queue_size=10
        )
        self.claw1_pub = rospy.Publisher(
            "/joint_6_controller/command", Float64, queue_size=10
        )
        self.claw2_pub = rospy.Publisher(
            "/joint_7_controller/command", Float64, queue_size=10
        )
        self.depth = rospy.Subscriber(
            "/camera/depth/points", PointCloud2, self.analysepc
        )
        rospy.spin()

    def analysepc(self, msg):
        # Convert PointCloud2 to a numpy array
        points = self.pointcloud2_to_xyz_array(msg)
        if len(points) == 0:
            return

        # Cluster points in 3D space
        clusters = list(self.get_all_clusters(points))

        # Analyse Each Cluster to see if its a pipe
        pipes = []
        for cluster in clusters:
            result = self.pipe_analysis(cluster)
            if len(result) == 0:
                continue
            pipes.append(result)
        print("Number of Pipes:", len(pipes))
        if len(pipes) == 0:
            return
        closest_distance = 100000
        for pipe in pipes:
            dist = math.sqrt(pipe[0][0] ** 2 + pipe[0][1] ** 2 + pipe[0][2] ** 2)
            if closest_distance > dist:
                closest_distance = dist
                closest_pipe = pipe

        midpoint = closest_pipe[0]
        angle = closest_pipe[1]
        tfmidpoint = [midpoint[2] + 0.12, -midpoint[1] + 0.04, -midpoint[0]]
        arm.move(
            tfmidpoint[0],
            tfmidpoint[1] + 0.1,
            tfmidpoint[2],
            0,
            0,
            0,
            angle,
            straight=False,
            open=True,
        )
        arm.move(
            tfmidpoint[0],
            tfmidpoint[1],
            tfmidpoint[2],
            0,
            0,
            0,
            angle,
            straight=False,
            open=True,
        )

    def pointcloud2_to_xyz_array(self, cloud_msg):
        points_list = []
        for p in pc2.read_points(
            cloud_msg, skip_nans=True, field_names=("x", "y", "z")
        ):
            if p[1] < 0.41:  # Only include points where y < 0.41
                points_list.append([p[0], p[1], p[2]])
        return np.array(points_list)

    def get_all_clusters(self, X: np.ndarray):
        db = DBSCAN(eps=0.1, min_samples=5).fit(X)
        labels = db.labels_

        unique_labels = set(labels)

        for k in unique_labels:
            if k == -1:  # Noise is labeled as -1
                continue

            class_member_mask = labels == k
            xy = X[class_member_mask]

            yield xy  # Yield the full cluster in 3D space

    # Outputs [midpoint, angle] if pipelike
    # Outputs [] if unpipelike
    def pipe_analysis(self, cluster):
        # Step 1: Check if the cluster has an acceptable height
        max_y = np.max(cluster[:, 1])
        min_y = np.min(cluster[:, 1])
        height = max_y - min_y

        # Assuming a threshold for pipe height (you can adjust this threshold)
        if height > 0.3:  # Example threshold, adjust as needed
            return []

        # Step 2: Represent the cluster with a 2D OBB using only x and z values
        points_2d = cluster[:, [0, 2]]  # Only consider x and z values
        obb_points = self.get_obb_points(points_2d)
        obb_points = np.array(obb_points)

        # Step 3: Find the midpoint of the OBB
        midpoint_xz = np.mean(obb_points, axis=0)
        midpoint_y = (max_y + min_y) / 2
        midpoint = [midpoint_xz[0], midpoint_y, midpoint_xz[1]]

        # Step 4: Check if the shape of the OBB is pipelike
        # Calculate width and length of the OBB
        side1 = np.linalg.norm(obb_points[0] - obb_points[1])
        side2 = np.linalg.norm(obb_points[1] - obb_points[2])

        # Facilitate angle finding
        if side1 > side2:
            length = side1
            width = side2
            dx2 = obb_points[0][0] - obb_points[1][0]
            dy2 = obb_points[0][1] - obb_points[1][1]
        else:
            length = side2
            width = side1
            dx2 = obb_points[1][0] - obb_points[2][0]
            dy2 = obb_points[1][1] - obb_points[2][1]
        # Assuming a threshold for pipe-like shape (width should be small relative to length)
        if width > 0.3:
            return []

        if length < 0.8:
            return []

        # Find angle between (point1, point2) and (0, 1) in x,z strictly
        # WristPitch Angle
        dx1 = 0
        dy1 = 1

        try:
            post = (dx1 * dx2 + dy1 * dy2) / (
                math.sqrt(dx1 * dx1 + dy1 * dy1) * math.sqrt(dx2 * dx2 + dy2 * dy2)
            )
        except ZeroDivisionError:
            if (dx1 * dx2 + dy1 * dy2) > 0:
                post = 1
            else:
                post = -1
        if post > 1:
            post = 1
        elif post < -1:
            post = -1

        # Find sign of angle using cross product
        cross = dx1 * dy2 - dx2 * dy1
        if cross >= 0:
            sign = -1
        else:
            sign = 1

        angle = math.acos(post) * sign
        if angle < -math.pi / 2:
            angle = angle + math.pi
        elif angle > math.pi / 2:
            angle = angle - math.pi

        return [midpoint, angle]

    def get_obb_points(self, points):
        # Convert points to a numpy array
        points = np.array(points, dtype=np.float32)

        # Compute the minimum area rectangle that encloses the points
        rect = cv2.minAreaRect(points)

        # Get the four corners of the rectangle
        obb_points = cv2.boxPoints(rect)

        return obb_points

    def move(
        self, x_f, y_f, z_f, x_i, y_i, z_i, alpha, straight, open
    ):  # [f] claw, [i] shoulder
        # Cylindrical Coordinates Relative Transformation
        x = x_f - x_i
        z = z_f - z_i

        a = math.sqrt(x**2 + z**2)
        b = y_f - y_i
        p = 0
        q = 0
        theta = math.atan2(z, x)

        l_1 = self.shoulder_elbow
        l_2 = self.elbow_claw

        # Verify if there is a solution
        d = math.sqrt((a - p) ** 2 + (b - q) ** 2)
        if d > l_1 + l_2:
            print("Pipe is too far")
            return
        if d < abs(l_1 - l_2):
            print("Pipe is too close")
            return
        print("Grabbing Closest Pipe")
        # Compute the solution
        l = (l_1**2 - l_2**2 + d**2) / (2 * d)

        h = math.sqrt(l_1**2 - l**2)

        X_1 = (l / d) * (a - p) + (h / d) * (b - q) + p
        X_2 = (l / d) * (a - p) - (h / d) * (b - q) + p

        Y_1 = (l / d) * (b - q) - (h / d) * (a - p) + q
        Y_2 = (l / d) * (b - q) + (h / d) * (a - p) + q

        if (Y_1) >= (Y_2):
            elbow = [X_1, Y_1]
        else:
            elbow = [X_2, Y_2]

        # Shoulder angle
        dx1 = 0
        dy1 = 1
        dx2 = elbow[0] - p
        dy2 = elbow[1] - q

        try:
            post = (dx1 * dx2 + dy1 * dy2) / (
                math.sqrt(dx1 * dx1 + dy1 * dy1) * math.sqrt(dx2 * dx2 + dy2 * dy2)
            )
        except ZeroDivisionError:
            if (dx1 * dx2 + dy1 * dy2) > 0:
                post = 1
            else:
                post = -1
        if post > 1:
            post = 1
        elif post < -1:
            post = -1

        # Find sign of angle using cross product
        cross = dx1 * dy2 - dx2 * dy1
        if cross >= 0:
            sign = -1
        else:
            sign = 1

        angle1 = math.acos(post) * sign

        # Elbow angle
        dx1 = elbow[0] - p
        dy1 = elbow[1] - q
        dx2 = a - elbow[0]
        dy2 = b - elbow[1]

        try:
            post = (dx1 * dx2 + dy1 * dy2) / (
                math.sqrt(dx1 * dx1 + dy1 * dy1) * math.sqrt(dx2 * dx2 + dy2 * dy2)
            )
        except ZeroDivisionError:
            if (dx1 * dx2 + dy1 * dy2) > 0:
                post = 1
            else:
                post = -1
        if post > 1:
            post = 1
        elif post < -1:
            post = -1

        # Find sign of angle using cross product
        cross = dx1 * dy2 - dx2 * dy1
        if cross >= 0:
            sign = -1
        else:
            sign = 1

        angle2 = math.acos(post) * sign - math.pi / 2

        # WristPitch Angle
        dx1 = 0
        dy1 = -1
        dx2 = a - elbow[0]
        dy2 = b - elbow[1]

        try:
            post = (dx1 * dx2 + dy1 * dy2) / (
                math.sqrt(dx1 * dx1 + dy1 * dy1) * math.sqrt(dx2 * dx2 + dy2 * dy2)
            )
        except ZeroDivisionError:
            if (dx1 * dx2 + dy1 * dy2) > 0:
                post = 1
            else:
                post = -1
        if post > 1:
            post = 1
        elif post < -1:
            post = -1

        # Find sign of angle using cross product
        cross = dx1 * dy2 - dx2 * dy1
        if cross >= 0:
            sign = -1
        else:
            sign = 1

        angle3 = math.acos(post) * sign

        if straight:
            angle3 = angle3 + math.pi / 2

        if open:
            angle4 = -0.2
        else:
            angle4 = 0.12

        if straight:
            angle5 = alpha
        else:
            angle5 = theta + alpha

        # Open or Close Claws First
        # Then wristroll
        for n in range(0, 70000):
            self.claw1_pub.publish(angle4)
            self.claw2_pub.publish(angle4)
            self.wristroll_pub.publish(angle5)

        # Move
        while True:
            self.wristroll_pub.publish(angle5)
            self.shoulder_pub.publish(angle1)
            self.elbow_pub.publish(angle2)
            self.tumor_pub.publish(theta)
            self.wristpitch_pub.publish(-angle3)
            self.claw1_pub.publish(angle4)
            self.claw2_pub.publish(angle4)
            self.wristroll_pub.publish(angle5)

    def open_or_grip(self, open):
        while True:
            if open:
                angle4 = -0.2
            else:
                angle4 = 0.12
            self.claw1_pub.publish(angle4)
            self.claw2_pub.publish(angle4)


if __name__ == "__main__":
    arm = ArmMoving()
    arm.pubs_and_subs()
