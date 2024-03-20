#!/usr/bin/env python3
"""
Path planning node
This node is responsible for starting the path planning module
"""
import rclpy
from rclpy.node import Node
from asurt_msgs.msg import LandmarkArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from src.utils.cone_types import ConeTypes
import numpy as np

from src.full_pipeline.full_pipeline import PathPlanner


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

    def __init__(self):
        super().__init__("planning_node")
        self.get_logger().info("Path Planner instantiated...")
        self.path = None
        self.cones = None
        self.carPosition = None
        self.carDirection = None
        self.subscriber1 = self.create_subscription(
            LandmarkArray, "/topic1", self.receive_from_perception, 10
        )
        self.subscriber2 = self.create_subscription(
            Odometry, "/topic2", self.receive_from_localization, 10
        )
        self.publisher = self.create_publisher(Path, "/topic3", 10)

    def receive_from_perception(self, msg: LandmarkArray):
        """
        Receives data from perception.

        Args:
            msg (LandmarkArray): The data received from perception.
        """
        # get cones_colors, cones_positions
        self.cones = [np.zeros((0, 2)) for _ in ConeTypes]
        for landmark in msg.landmarks:
            if landmark.identifier == 0:
                self.cones[ConeTypes.BLUE] = np.vstack(
                    (self.cones[ConeTypes.BLUE], [landmark.position.x, landmark.position.y])
                )
            elif landmark.identifier == 1:
                self.cones[ConeTypes.YELLOW] = np.vstack(
                    (self.cones[ConeTypes.YELLOW], landmark.position)
                )
            else:
                self.cones[ConeTypes.UNKNOWN] = np.vstack(
                    (self.cones[ConeTypes.UNKNOWN], landmark.position)
                )

        self.send_to_control()

    def receive_from_localization(self, msg: Odometry):
        """
        Receives data from localization.

        Args:
            msg (Odometry): The data received from localization.
        """
        # get car_position, car_direction
        pose = msg.pose.pose
        self.carPosition = [pose.position.x, pose.position.y]
        self.carDirection = [pose.orientation.x, pose.orientation.y]

    def send_to_control(self):
        """
        Sends the calculated path to control.
        """
        pathPlanner = PathPlanner()
        self.path = pathPlanner.calculatePathInGlobalFrame(
            vehiclePosition=self.carPosition, vehicleDirection=self.carDirection, cones=self.cones
        )
        if self.path is not None:
            timestamp = rclpy.time.now()
            path = Path()
            path.header.stamp = timestamp
            path.header.frame_id = "path"

            for dataPoint in self.path:
                pose = Pose()
                pose.position.x = dataPoint[0]
                pose.position.y = dataPoint[1]

                poseStamped = PoseStamped()
                poseStamped.pose = pose
                poseStamped.header.stamp = timestamp
                poseStamped.header.frame_id = "path"

                path.poses.append(poseStamped)

            self.publisher.publish(path)

            self.get_logger().info("Path Sent...")


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
