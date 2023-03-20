"""
Smoreo ros wrapper
"""
from typing import Dict, Any, Union
import rospy
import numpy as np
import tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes
from tf_helper.MarkerViz import MarkerViz
from smoreo.smoreo import Smoreo
from smoreo.utils import processBboxes


class SmoreoRosWrapper:
    """
    Ros wrapper for smoreo system
    """

    def __init__(self) -> None:
        self.params: Dict[str, Any]
        self.publishers: Dict[str, rospy.Publisher] = {}
        self.boundingBoxes: BoundingBoxes
        self.smoreo: Smoreo
        self.useConeBase: bool
        self.inTuning: bool
        self.markerViz: MarkerViz

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
                "/smoreo/camera_height_from_ground",
                "/smoreo/cone_height",
                "/smoreo/camera_info",
                "/smoreo/hardcode_params",
            ]:
                assert rospy.has_param(key)

            if rospy.get_param("/smoreo/hardcode_params"):
                assert rospy.has_param("/smoreo/cx")
                assert rospy.has_param("/smoreo/cy")
                assert rospy.has_param("/smoreo/f")
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
        if rospy.get_param("/smoreo/hardcode_params"):
            fInPixels = rospy.get_param("smoreo/f")
            cameraCx = rospy.get_param("smoreo/cx")
            camerCy = rospy.get_param("smoreo/cy")
        else:
            cameraInfo = rospy.wait_for_message(rospy.get_param("/smoreo/camera_info"), CameraInfo)
            fInPixels = cameraInfo.K[0]
            cameraCx = cameraInfo.K[2]
            camerCy = cameraInfo.K[5]

        coneHeight = rospy.get_param("smoreo/cone_height", 0.4)

        cutOffY = rospy.get_param("smoreo/cut_off_y")
        listener = tf.TransformListener()
        listener.waitForTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(), rospy.Duration(4.0)
        )
        _, worldToCameraRotation = listener.lookupTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(0)
        )
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
            "camera_height_from_ground": rospy.get_param("smoreo/camera_height_from_ground"),
            "cut_off_y": float(cutOffY),
            "cone_height": coneHeight,
        }
        return params

    def createPublishers(self) -> None:
        """
        Create needed publishers, for now only for the predicted landmarks, and the marker array.
        """
        try:
            assert rospy.has_param("/smoreo/predicted_landmarks")
            assert rospy.has_param("/smoreo/predicted_markers")
        except Exception as exp:
            errMsg = "smoreo: ensure all the required topics for\n \
                         publishing are provided in ros server\n \
                        - predicted_landmarks\n \
                        - predicted_markers"
            raise ValueError(errMsg) from exp

        self.publishers["landmarkPub"] = rospy.Publisher(
            rospy.get_param("/smoreo/predicted_landmarks"), LandmarkArray, queue_size=10
        )
        self.publishers["markersPub"] = rospy.Publisher(
            rospy.get_param("/smoreo/predicted_markers"), MarkerArray, queue_size=10
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
            if self.useConeBase:
                predictedLandmarks = self.smoreo.predictWithBase(self.boundingBoxes)
            else:
                predictedLandmarks = self.smoreo.predictWithTop(self.boundingBoxes)

            try:
                predictedLandmarks.header.stamp = rospy.Time.now()
            except rospy.exceptions.ROSInitException:
                rospy.logwarn("ROS not initialized, using time 0 for header")

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
        coneRadius = 0.228
        self.markerViz = MarkerViz(coneRadius, self.params["cone_height"])
        self.boundingBoxes = None
        self.smoreo = Smoreo(self.params)
        if "/smoreo/bounding_boxes" not in rospy.get_param_names():
            raise ValueError("smoreo: bounding boxes topic not provided")
        rospy.Subscriber(
            rospy.get_param("/smoreo/bounding_boxes"), BoundingBoxes, self.setBoundingBoxes
        )
