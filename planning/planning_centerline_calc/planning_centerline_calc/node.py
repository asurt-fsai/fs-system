#!/usr/bin/env python3
"""
Path planning node
This node is responsible for starting the path planning module
"""
import rclpy
from rclpy.node import Node
from asurt_msgs.msg import LandmarkArray,Landmark
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from typing import Any
from tf_transformations import euler_from_quaternion

from src.full_pipeline.full_pipeline import PathPlanner
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import angleFrom2dVector
import matplotlib.pyplot as plt

class PlanningNode(Node):
    """
    A class representing a planning node.

    This node is responsible for receiving data from perception and localization,
    calculating a path based on the received data, and sending the path to control.

    Attributes:
        path (list): The calculated path.
        cones (list): The positions of cones detected by perception.
        carPosition (list): The position of the car.
        carDirection (list): The direction of the car.
        subscriber1 (Subscriber): The subscriber for receiving data from perception.
        subscriber2 (Subscriber): The subscriber for receiving data from localization.
        publisher (Publisher): The publisher for sending the calculated path.

    Methods:
        receive_from_perception: Receives data from perception.
        receive_from_localization: Receives data from localization.
        send_to_control: Sends the calculated path to control.
    """

    def __init__(self) -> None:
        super().__init__("planning_node")
        self.get_logger().info("Path Planner instantiated...")
        self.path = None
        self.cones = None
        self.carPosition = np.array([0, 0])
        self.carDirection = 0
        self.pathPlanner = PathPlanner()
        self.subscriber1 = self.create_subscription(
            LandmarkArray, "/Landmarks/Observed", self.receiveFromPerception, 10
        )
        self.subscriber2 = self.create_subscription(
            Odometry, "/carmaker/Odometry", self.receiveFromLocalization, 10
        )
        plt.ion()
        self.publisher = self.create_publisher(Path, "/topic3", 10)
        self.figure, self.ax = plt.subplots()
        self.figure.show()

    def receiveFromPerception(self, msg: LandmarkArray) -> None:
        """
        Receives data from perception.

        Args:
            msg (LandmarkArray): The data received from perception.
        """
        # get cones_colors, cones_positions
        self.cones = [np.zeros((0, 2)) for _ in ConeTypes]
        for landmark in msg.landmarks:
            if landmark.type == Landmark.BLUE_CONE:
                self.cones[ConeTypes.BLUE] = np.vstack(
                    (self.cones[ConeTypes.BLUE], np.array([landmark.position.x, landmark.position.y]))
                )
            elif landmark.type == Landmark.YELLOW_CONE:
                self.cones[ConeTypes.YELLOW] = np.vstack(
                    (self.cones[ConeTypes.YELLOW], np.array([landmark.position.x, landmark.position.y]))
                )
            else:
                self.cones[ConeTypes.UNKNOWN] = np.vstack(
                    (self.cones[ConeTypes.UNKNOWN], np.array([landmark.position.x, landmark.position.y]))
                )

        self.sendToControl()

    def receiveFromLocalization(self, msg: Odometry) -> None:
        """
        Receives data from localization.

        Args:
            msg (Odometry): The data received from localization.
        """
        # get car_position, car_direction
        pose = msg.pose.pose
        orientation_q = pose.orientation
        self.carPosition = np.array([pose.position.x, pose.position.y])
        
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.carDirection = yaw
        # print(self.carDirection)

    def sendToControl(self) -> None:
        """
        Sends the calculated path to control.
        """
        
        if self.carDirection is None:
            return
        self.path = self.pathPlanner.calculatePathInGlobalFrame(
            vehiclePosition=self.carPosition, vehicleDirection=self.carDirection, cones=self.cones
        )
        if self.path is not None:
            timestamp = self.get_clock().now().to_msg()
            path = Path()
            path.header.stamp = timestamp
            path.header.frame_id = "map"

            for dataPoint in self.path:
                pose = Pose()
                pose.position.x = dataPoint[0]
                pose.position.y = dataPoint[1]

                poseStamped = PoseStamped()
                poseStamped.pose = pose
                poseStamped.header.stamp = timestamp
                poseStamped.header.frame_id = "map"

                path.poses.append(poseStamped)

            self.publisher.publish(path)
            self.get_logger().info("Path Sent...")
            self.ax.clear()
            self.ax.scatter(self.cones[ConeTypes.UNKNOWN][:,0],self.cones[ConeTypes.UNKNOWN][:,1],c="black")
            self.ax.scatter(self.cones[ConeTypes.BLUE][:,0],self.cones[ConeTypes.BLUE][:,1],c="blue")
            self.ax.scatter(self.cones[ConeTypes.YELLOW][:,0],self.cones[ConeTypes.YELLOW][:,1],c="yellow")
            self.ax.scatter(self.carPosition[0], self.carPosition[1], c="red")
            self.ax.scatter(self.path[:,0], self.path[:,1])
            plt.pause(0.001)


def main(args: Any = None) -> None:
    """
    Initializes ROS, creates PlanningNode, spins, & shuts down.
    """
    rclpy.init(args=args)
    node = PlanningNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
