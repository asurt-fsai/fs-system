"""
Smoreo ros wrapper
"""
from typing import Dict, Any, Union
# import rospy
import numpy as np
import tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes
from tf_helper.MarkerViz import MarkerViz
from smoreo.smoreo import Smoreo
from smoreo.utils import processBboxes
import rclpy


class SmoreoRosWrapper:
    """
    Ros wrapper for smoreo system
    """

    def __init__(self,node) -> None:
        self.params: Dict[str, Any]
        self.publishers: Dict[str, rclpy.publisher] = {}
        self.boundingBoxes: BoundingBoxes
        self.smoreo: Smoreo
        self.useConeBase: bool
        self.inTuning: bool
        self.markerViz: MarkerViz
        self.node = node

    def getParams(self) -> Dict[str, Any]:
        """
        Get params of the system from the ros parameters server

        Parameters:
        ------------
        None
        Returns:
        -----------
        Dict[str,Any]: parameters obtained from the ros parameter server.
        """
        try:
            for key in [
                "/smoreo/cut_off_y",
                "/physical/camera_height_from_ground",
                "/physical/cone_height",
                "/smoreo/camera_info",
                "/smoreo/hardcode_params",
            ]:
                assert self.node.has_parameter(key)

            if self.node.get_parameter("/smoreo/hardcode_params"):
                assert self.node.has_parameter("/smoreo/cx")
                assert self.node.has_parameter("/smoreo/cy")
                assert self.node.has_parameter("/smoreo/f")
        except Exception as exp:
            errMsg = "smoreo: ensure all the required parameters are provided in ros server\n \
                       - cut_off_y \n\
                       - camera_height_from_ground \n\
                       - cone_height \n\
                       - camera_info \n\
                       - hardcode_params \n\
                        - cx \n\
                        - cy \n\
                        - f"
            raise TypeError(errMsg) from exp
        if self.node.get_parameter("/smoreo/hardcode_params"):
            fInPixels = self.node.get_parameter("smoreo/f")
            cameraCx = self.node.get_parameter("smoreo/cx")
            camerCy = self.node.get_parameter("smoreo/cy")
        else:
            cameraInfo = rclpy.wait_for_message(self.node.get_parameter("/smoreo/camera_info"), CameraInfo)
            fInPixels = cameraInfo.K[0]
            cameraCx = cameraInfo.K[2]
            camerCy = cameraInfo.K[5]

        coneHeight = self.node.get_parameter("physical/cone_height", 0.4)

        cutOffY = self.node.get_parameter("smoreo/cut_off_y")

        worldToCameraRotation = [0.5, -0.5, 0.5, 0.5]
        params = {
            "cx": cameraCx,
            "cy": camerCy,
            "f": fInPixels,
            "k": np.array(
                [[fInPixels, 0.0, cameraCx], [0.0, fInPixels, camerCy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            "worldCords_inCamera": np.array(
                tf.transformations.quaternion_matrix(worldToCameraRotation)[:3, :3], dtype=int
            ),
            "camera_height_from_ground": self.node.get_parameter("/physical/camera_height_from_ground"),
            "cut_off_y": float(cutOffY),
            "cone_height": coneHeight,
        }
        return params

    def createPublishers(self) -> None:
        """
        Create needed publishers, for now only for the predicted landmarks, and the marker array.
        """
        try:
            assert self.node.has_parameter("/smoreo/predicted_landmarks")
            assert self.node.has_parameter("/smoreo/predicted_markers")
        except Exception as exp:
            errMsg = "smoreo: ensure all the required topics for\n \
                         publishing are provided in ros server\n \
                        - predicted_landmarks\n \
                        - predicted_markers"
            raise ValueError(errMsg) from exp

        self.publishers["landmarkPub"] = self.node.create_publisher(LandmarkArray,
            self.node.get_parameter("/smoreo/predicted_landmarks"), 10
        )
        self.publishers["markersPub"] = self.node.create_publisher(MarkerArray,
            self.node.get_parameter("/smoreo/predicted_markers"), 10
        )

    def setBoundingBoxes(self, boundingBoxes: BoundingBoxes) -> None:
        """
        Main callback function for the system that accepts bounding boxes from the subscriber.

        Parameters:
        ------------
        boundingBoxes: BoundingBoxes

        Returns:
        -----------
        None
        """
        self.boundingBoxes = processBboxes(boundingBoxes)

    def run(self) -> Union[LandmarkArray, None]:
        """
        Run the smoreo system.
        """
        if self.boundingBoxes is not None:
            # Get parameters every run if in tuning mode
            if self.inTuning:
                newParams = self.getParams()
                self.smoreo.updateParams(newParams)
            predictedLandmarks = self.smoreo.predictLandmarks(self.boundingBoxes)
            self.boundingBoxes = None

            try:
                predictedLandmarks.header.stamp = rclpy.clock.Clock.now()
            except rclpy.exceptions.NotInitializedException:
                rclpy.logging.warn("ROS not initialized, using time 0 for header")

            self.publishers["landmarkPub"].publish(predictedLandmarks)
            self.publishers["markersPub"].publish(self.markerViz.conesToMarkers(predictedLandmarks))
            return predictedLandmarks
        return None

    def start(self, useConeBase: bool, inTuning: bool) -> None:
        """
        start the smoreo system by creating publishers, subscribers and the smoreo object.
        """
        self.useConeBase = useConeBase
        self.inTuning = inTuning
        self.createPublishers()
        self.params = self.getParams()
        coneRadius = self.node.get_parameter("/physical/cone_radius")
        coneHeight = self.node.get_parameter("/physical/cone_height")
        lidarHeight = self.node.get_parameter("/physical/lidar_height")
        self.markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)
        self.boundingBoxes = None
        self.smoreo = Smoreo(self.params, self.node.get_parameter("/smoreo/camera_frame"))
        if "/smoreo/bounding_boxes" not in self.node.get_parameter_names():
            raise ValueError("smoreo: bounding boxes topic not provided")
        self.subscription= self.create_subscription( BoundingBoxes,
            self.node.get_parameter("/smoreo/bounding_boxes"), self.setBoundingBoxes
        )
