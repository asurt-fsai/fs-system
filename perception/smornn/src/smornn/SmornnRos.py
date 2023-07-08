"""
A ros wrapper for the Smornn class
"""
from typing import Dict, Any, Optional

import rospy
import numpy as np
from asurt_msgs.msg import LandmarkArray

from tf_helper.TFHelper import TFHelper
from tf_helper.utils import parseLandmarks, createLandmarkMessage
from .Smornn import Smornn


class SmornnRos(Smornn):
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

    def __init__(
        self,
        publishers: Dict[str, rospy.Publisher],
        markerViz: Any,
        frameId: str,
        *args: Any,
        **kwargs: Any,
    ):
        super().__init__(*args, **kwargs)
        self.publishers = publishers
        self.markerViz = markerViz
        self.frameId = frameId
        self.tfHelper = TFHelper("smornn")

        # Parameter Validation
        try:
            for key, value in publishers.items():
                assert isinstance(value, rospy.Publisher)

            for key in ["detected", "detected_markers"]:
                assert key in publishers
        except Exception as exp:
            errMsg = "Smornn: ensure the all the publishers required are provided \n \
                       - detected \n \
                       - detected_markers"
            raise TypeError(errMsg) from exp

    def lidarCallback(self, cones: LandmarkArray) -> None:
        cones = self.tfHelper.transformLandmarkArrayMsg(cones, self.frameId)
        cones = parseLandmarks(cones)
        if len(cones) == 0:
            super().lidarCallback([])
        else:
            super().lidarCallback(
                cones[:, :2]
            )  # Use only x, y of the cones without color for lidar

    def smoreoCallback(self, cones: LandmarkArray) -> None:
        cones = self.tfHelper.transformLandmarkArrayMsg(cones, self.frameId)
        cones = parseLandmarks(cones)
        super().smoreoCallback(cones)

    def run(self) -> Optional[LandmarkArray]:
        """
        Runs the smornn algorithm and publishes the results
        """
        cones = super().run()
        if cones is None:
            return None

        coneProbs = np.ones((cones.shape[0], 1))
        landmarks = createLandmarkMessage(cones[:, :2], cones[:, 2], coneProbs, self.frameId)
        self.publishers["detected"].publish(landmarks)
        detectedMarkers = self.markerViz.conesToMarkers(landmarks)
        self.publishers["detected_markers"].publish(detectedMarkers)
        return landmarks
