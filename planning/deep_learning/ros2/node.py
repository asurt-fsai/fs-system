#!/usr/bin/env python3
"""Importing necessary libraries for the ros2 node"""
from typing import Any
import sys
from tf_helper import TFHelper
import rclpy
from rclpy.node import Node
import torch
from asurt_msgs.msg import LandmarkArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from planning_deep_learning import model
import numpy as np

sys.path.append("/home/ubuntu/ros2_ws/src/tf_helper/")


class PlanningDlNode(Node):  # type: ignore
    """
    ROS2 node for deep learning-based path planning.

    This node receives landmarks from perception, processes them, and uses a deep learning model
    to predict a path based on the processed landmarks. If a valid path is obtained, it publishes
    the path as a ROS2 Path message.

    Attributes:
        model: The deep learning model used for path prediction.
        path: The predicted path.
        conesList: The processed list of cones.
        tfhelper: The TFHelper object for transforming messages.
        subscriber1: The subscriber for receiving landmarks from perception.
        publisher: The publisher for publishing the predicted path.

    Methods:
        __init__: Initializes the PlanningDlNode object.
        receiveFromPerception: Receives a message from perception and processes the landmarks.
        sendToControl: Sends the predicted path to the control system.
    """

    def __init__(self) -> None:
        super().__init__("planning_dl")
        self.get_logger().info("DL Path Planner instantiated...")
        modelPath = ("./model_1000_epoch.pt")
        self.model = model.createModel(modelPath)
        self.path = None
        self.conesList = Any
        self.tfhelper = TFHelper.TFHelper(self)
        # self.carPosition = None
        self.subscriber1 = self.create_subscription(
            LandmarkArray, "/Landmarks/Observed", self.receive_from_perception, 10
        )
        self.publisher = self.create_publisher(Path, "/dl_path", 10)

    def receiveFromSlam(self, msg: LandmarkArray) -> None:
        """
        Receives a message from perception and processes the landmarks.

        Args:
            msg (LandmarkArray): The message containing the landmarks.

        Returns:
            None
        """

        self.get_logger().info("Message received...")

        msg = self.tfhelper.transformMsg(msg, "Obj_F")
        self.conesList = []

        # Process incoming landmarks
        for landmark in msg.landmarks:
            cones = []
            # Encode color one-hot
            colorOneHot = [0, 0, 0, 0]  # [blue, yellow, orange, unknown]
            # color_one_hot = [0, 0, 0]  # [blue, yellow, orange]
            if landmark.identifier == 0:
                colorOneHot[0] = 1
                cones.append(np.array([landmark.position.x, landmark.position.y] + colorOneHot))
            elif landmark.identifier == 1:
                colorOneHot[1] = 1
                cones.append(np.array([landmark.position.x, landmark.position.y] + colorOneHot))
            elif landmark.identifier == 3:
                colorOneHot[2] = 1
                cones.append(np.array([landmark.position.x, landmark.position.y] + colorOneHot))
            else:
                colorOneHot[3] = 1
                cones.append(np.array([landmark.position.x, landmark.position.y] + colorOneHot))

            # Vertically stack the arrays
            self.conesList = np.vstack(cones)
        # Convert to torch tensor
        # Convert list of lists to single NumPy array
        conesArray = np.array(self.conesList, dtype=np.float32)  # Assuming all elements are numeric
        # Convert NumPy array to tensor
        self.conesList = torch.tensor(conesArray)
        self.sendToControl()

    def sendToControl(self) -> None:
        """
        Sends the predicted path to the control system.

        This method uses the model to predict the path based on the cones list.
        If a valid path is obtained, it creates a ROS2 Path message and publishes it.
        The path is constructed by converting each data point in the predicted path
        to a PoseStamped message and appending it to the Path message.

        Returns:
            None
        """
        self.path = self.model.predict(self.conesList)

        if self.path is not None:
            timestamp = self.get_clock().now().to_msg()
            path = Path()
            path.header.stamp = timestamp
            path.header.frame_id = "dl_path"

            for dataPoint in self.path:
                pose = Pose()
                pose.position.x = float(dataPoint[0])
                pose.position.y = float(dataPoint[1])

                poseStamped = PoseStamped()
                poseStamped.pose = pose
                poseStamped.header.stamp = timestamp
                poseStamped.header.frame_id = "map"

                path.poses.append(poseStamped)

            self.publisher.publish(path)

            self.get_logger().info("DL Path Sent...")


def main(args: None = None) -> None:
    """
    Entry point of the program.

    Args:
        args: Command-line arguments (default: None).
    """
    rclpy.init(args=args)
    node = PlanningDlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
