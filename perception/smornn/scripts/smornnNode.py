#!/usr/bin/python3
"""
A ros wrapper for the Smornn class
"""
from typing import Optional

import numpy as np
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray

from rclpy.node import Node
import rclpy
from tf_helper.TFHelper import TFHelper
from tf_helper.utils import Utils
from tf_helper.StatusPublisher import StatusPublisher
from tf_helper.MarkerViz import MarkerViz
from Smornn import Smornn

# mypy: disable-error-code="misc"
class SmornnNode(Node):
    """
    A ros wrapper for the Smornn class

    Parameters
    ----------
    publishers: Dict[str, rospy.Publisher]
        A dictionary of publishers, the keys are the names of the publishers
        and the values are the publishers themselves
    markerViz: MarkerViz
        A markerViz object used to publish the detections as MarkerArrays for visualization
    """

    def __init__(self) -> None:
        """
        Init function
        """
        super().__init__("smornnNode")

        self.declare_parameter("/physical/lidar_height", rclpy.Parameter.Type.DOUBLE)
        lidarHeight = (
            self.get_parameter("/physical/lidar_height").get_parameter_value().double_value
        )
        self.declare_parameter("/physical/cone_height", rclpy.Parameter.Type.DOUBLE)
        coneHeight = self.get_parameter("/physical/cone_height").get_parameter_value().double_value
        self.declare_parameter("/physical/cone_radius", rclpy.Parameter.Type.DOUBLE)
        coneRadius = self.get_parameter("/physical/cone_radius").get_parameter_value().double_value
        self.markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)

        self.declare_parameter("/perception/smornn/frame_id", rclpy.Parameter.Type.STRING)
        self.frameId = (
            self.get_parameter("/perception/smornn/frame_id").get_parameter_value().string_value
        )

        self.tfHelper = TFHelper(self)

        self.declare_parameter("/perception/smornn/min_dist_neighbor", rclpy.Parameter.Type.DOUBLE)
        minDistNeighbor = (
            self.get_parameter("/perception/smornn/min_dist_neighbor")
            .get_parameter_value()
            .double_value
        )
        self.smornn = Smornn(minDistNeighbor)
        self.utils = Utils(self)

        self.declare_parameter("/perception/smornn/detected", rclpy.Parameter.Type.STRING)
        publishTopic = (
            self.get_parameter("/physical/cone_height").get_parameter_value().string_value
        )
        self.declare_parameter("/perception/smornn/detected_markers", rclpy.Parameter.Type.STRING)
        markerTopic = (
            self.get_parameter("/perception/smornn/detected_markers")
            .get_parameter_value()
            .string_value
        )
        self.detected = self.create_publisher(LandmarkArray, publishTopic, 1)
        self.detectedMarkers = self.create_publisher(MarkerArray, markerTopic, 1)

        self.declare_parameter("/perception/lidar/detected", rclpy.Parameter.Type.STRING)
        lidarTopic = (
            self.get_parameter("/perception/lidar/detected").get_parameter_value().string_value
        )

        self.declare_parameter("/perception/smoreo/detected", rclpy.Parameter.Type.STRING)
        smoreoTopic = (
            self.get_parameter("/perception/smoreo/detected").get_parameter_value().string_value
        )

        self.create_subscription(LandmarkArray, lidarTopic, self.lidarCallback, 1)
        self.create_subscription(LandmarkArray, smoreoTopic, self.smoreoCallback, 1)

    def lidarCallback(self, cones: LandmarkArray) -> None:
        """
        Callback function for lidar
        """
        cones = self.tfHelper.transformLandmarkArrayMsg(cones, self.frameId)
        cones = self.utils.parseLandmarks(cones)
        if len(cones) == 0:
            self.smornn.lidarCallback([])
        else:
            self.smornn.lidarCallback(
                cones[:, :2]
            )  # Use only x, y of the cones without color for lidar

    def smoreoCallback(self, cones: LandmarkArray) -> None:
        """
        Callback function for smoreo
        """
        cones = self.tfHelper.transformLandmarkArrayMsg(cones, self.frameId)
        cones = self.utils.parseLandmarks(cones)
        self.smornn.smoreoCallback(cones)

    def run(self) -> Optional[LandmarkArray]:
        """
        Runs the smornn algorithm and publishes the results
        """
        cones = self.smornn.run()
        if cones is None:
            return None

        coneProbs = np.ones((cones.shape[0], 1))

        landmarks = self.utils.createLandmarkMessage(
            cones[:, :2], cones[:, 2], coneProbs, self.frameId
        )
        self.detected.publish(landmarks)
        detectedMarkers = self.markerViz.conesToMarkers(landmarks)
        self.detectedMarkers.publish(detectedMarkers)

        return landmarks


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rclpy.init()
    smornn = SmornnNode()
    status = StatusPublisher(smornn, "/status/smornn")

    status.starting()

    # Publish heartbeat to show the module is ready
    status.ready()

    # Main loop
    rate = smornn.create_rate(100)
    while rclpy.ok():
        rate.sleep()
        out = smornn.run()
        if out is None:
            continue

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    main()
