#!/usr/bin/env python3
"""
Path planning node
This node is responsible for starting the path planning module
"""

from typing import Any, List
import rclpy
from rclpy.node import Node
from asurt_msgs.msg import LandmarkArray,Landmark
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from tf_transformations import euler_from_quaternion

from src.full_pipeline.full_pipeline import PathPlanner, ParametersState
from src.utils.cone_types import ConeTypes
from src.types_file import FloatArray


class PlanningNode(Node): # pylint: disable=too-many-instance-attributes
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
        self.path: FloatArray = np.zeros((0, 2))
        self.cones: List[FloatArray] = [np.zeros((0, 2)) for _ in ConeTypes]
        self.carPosition = np.array([0, 0])
        self.carDirection = np.float_(0.0)

        self.publisher: rclpy.publisher.Publisher

        self.declareParameters()
        self.setParameters()
        self.initPubAndSub()

    def declareParameters(self) -> None:
        """
        Declare the parameters for the planning node.
        """
        self.declare_parameter(
            "planning.sorting.thresholdDirectionalAngle", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.sorting.thresholdAbsoluteAngle", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.sorting.maxNNeighbors", rclpy.Parameter.Type.INTEGER
            )
        self.declare_parameter(
            "planning.sorting.maxDist", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.sorting.maxDistToFirst", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.sorting.maxLength", rclpy.Parameter.Type.INTEGER
            )

        self.declare_parameter(
            "planning.matching.minTrackWidth", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.matching.maxSearchRange", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.matching.maxSearchAngle", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.matching.matchesShouldBeMonotonic", rclpy.Parameter.Type.BOOL
            )

        self.declare_parameter(
            "planning.path.maximalDistanceForValidPath", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.path.mpcPathLength", rclpy.Parameter.Type.DOUBLE
            )
        self.declare_parameter(
            "planning.path.mpcPredictionHorizon", rclpy.Parameter.Type.INTEGER
            )

        self.declare_parameter("planning.topics.frame_id", rclpy.Parameter.Type.STRING)
        self.declare_parameter("planning.topics.pathTopic", rclpy.Parameter.Type.STRING)
        self.declare_parameter("planning.topics.odometryTopic", rclpy.Parameter.Type.STRING)
        self.declare_parameter("planning.topics.conesTopic", rclpy.Parameter.Type.STRING)

    def setParameters(self) -> None:
        """
        Get parameters from the parameter server and set them to their respective varibles
        """
        thresholdDirectionalAngle = (
            np.deg2rad(
                self.get_parameter(
                    "planning.sorting.thresholdDirectionalAngle"
                    ).get_parameter_value().double_value
                )
            )
        thresholdAbsoluteAngle = (
            np.deg2rad(
                self.get_parameter(
                    "planning.sorting.thresholdAbsoluteAngle"
                    ).get_parameter_value().double_value
                )
            )
        maxNNeighbors = (
            self.get_parameter(
                "planning.sorting.maxNNeighbors"
                ).get_parameter_value().integer_value
            )
        maxDist = (
            self.get_parameter(
                "planning.sorting.maxDist"
                ).get_parameter_value().double_value
            )
        maxDistToFirst = (
            self.get_parameter(
                "planning.sorting.maxDistToFirst"
                ).get_parameter_value().double_value
            )
        maxLength = (
            self.get_parameter(
                "planning.sorting.maxLength"
                ).get_parameter_value().integer_value
            )

        minTrackWidth = (
            self.get_parameter(
                "planning.matching.minTrackWidth"
                ).get_parameter_value().double_value
            )
        maxSearchRange = (
            self.get_parameter(
                "planning.matching.maxSearchRange"
                ).get_parameter_value().double_value
            )
        maxSearchAngle = (
                np.deg2rad(
                    self.get_parameter(
                        "planning.matching.maxSearchAngle"
                        ).get_parameter_value().double_value
                    )
            )
        matchesShouldBeMonotonic = (
            self.get_parameter(
                "planning.matching.matchesShouldBeMonotonic"
                ).get_parameter_value().bool_value
            )

        maximalDistanceForValidPath = (
            self.get_parameter(
                "planning.path.maximalDistanceForValidPath"
                ).get_parameter_value().double_value
            )
        mpcPathLength = (
            self.get_parameter(
                "planning.path.mpcPathLength"
                ).get_parameter_value().double_value
            )
        mpcPredictionHorizon = (
            self.get_parameter(
                "planning.path.mpcPredictionHorizon"
                ).get_parameter_value().integer_value
            )

        self.params = ParametersState(
            thresholdDirectionalAngle = thresholdDirectionalAngle,
            thresholdAbsoluteAngle = thresholdAbsoluteAngle,
            maxNNeighbors = maxNNeighbors,
            maxDist = maxDist,
            maxDistToFirst = maxDistToFirst,
            maxLength = maxLength,

            minTrackWidth = minTrackWidth,
            maxSearchRange = maxSearchRange,
            maxSearchAngle = maxSearchAngle,
            matchesShouldBeMonotonic = matchesShouldBeMonotonic,

            maximalDistanceForValidPath = maximalDistanceForValidPath,
            mpcPathLength = mpcPathLength,
            mpcPredictionHorizon = mpcPredictionHorizon
        )

        self.pathPlanner = PathPlanner(self.params)

        self.frameId = (
            self.get_parameter("planning.topics.frame_id").get_parameter_value().string_value
        )

    def initPubAndSub(self) -> None:
        """
        Initialize Publishers and subscribers for planning node

        Parameters
        ----------

        pathTopic: str
            The topic to publish the path
        
        odometryTopic: str
            The topic to subscribe to the odometry information comming from SLAM

        conesTopic: str
            The topic to subscribe to the cones information comming from SLAM
        """
        pathTopic = (
            self.get_parameter("planning.topics.pathTopic").get_parameter_value().string_value
        )
        odometryTopic = (
            self.get_parameter("planning.topics.odometryTopic").get_parameter_value().string_value
        )
        conesTopic = (
            self.get_parameter("planning.topics.conesTopic").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(Path, pathTopic, 10)
        self.subscriber1 = self.create_subscription(
            LandmarkArray, conesTopic, self.receiveFromPerception, 10
        )
        self.subscriber2 = self.create_subscription(
            Odometry, odometryTopic, self.receiveFromLocalization, 10
        )
        

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
                    (self.cones[ConeTypes.BLUE],
                     np.array([landmark.position.x,
                               landmark.position.y]))
                )
            elif landmark.type == Landmark.YELLOW_CONE:
                self.cones[ConeTypes.YELLOW] = np.vstack(
                    (self.cones[ConeTypes.YELLOW],
                     np.array([landmark.position.x,
                               landmark.position.y]))
                )
            else:
                self.cones[ConeTypes.UNKNOWN] = np.vstack(
                    (self.cones[ConeTypes.UNKNOWN],
                     np.array([landmark.position.x,
                               landmark.position.y]))
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
        orientationQ = pose.orientation
        self.carPosition = np.array([pose.position.x, pose.position.y])

        orientationList = [orientationQ.x, orientationQ.y, orientationQ.z, orientationQ.w]
        (_, _, yaw) = euler_from_quaternion(orientationList)
        self.carDirection = yaw


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
            path.header.frame_id = self.frameId

            for dataPoint in self.path:
                pose = Pose()
                pose.position.x = dataPoint[0]
                pose.position.y = dataPoint[1]

                poseStamped = PoseStamped()
                poseStamped.pose = pose
                poseStamped.header.stamp = timestamp
                poseStamped.header.frame_id = self.frameId

                path.poses.append(poseStamped)

            self.publisher.publish(path)
            self.get_logger().info("Path sent to control")


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
