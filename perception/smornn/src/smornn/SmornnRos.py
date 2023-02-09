"""
A ros wrapper for the Smornn class
"""
from typing import Dict, Any

import rospy
import numpy as np
import numpy.typing as npt
from asurt_msgs.msg import LandmarkArray, Landmark

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
        *args: Any,
        **kwargs: Any,
    ):
        super().__init__(*args, **kwargs)
        self.publishers = publishers
        self.markerViz = markerViz

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

    def toConeArray(
        self, landmarks: LandmarkArray, addType: bool = False
    ) -> npt.NDArray[np.float64]:
        """
        Converts a LandmarkArray to a numpy array of cones

        Parameters
        ----------
        landmarks : LandmarkArray
            Landmark array to convert
        addType : bool, by default False
            Whether to add the type and type probability to the cones

        Returns
        -------
        npt.NDArray[np.float64]
            Numpy array of cones, each row is a cone and contains either
            [x, y] or [x, y, color, color_prob]
        """
        cones = []
        for landmark in landmarks.landmarks:
            toAdd = [landmark.position.x, landmark.position.y]
            if addType:
                toAdd.extend([landmark.type, 0])
            cones.append(toAdd)
        return np.array(cones)

    def toLandmarkArray(self, cones: npt.NDArray[np.float64]) -> LandmarkArray:
        """
        Converts a numpy array of cones to a LandmarkArray

        Parameters
        ----------
        cones : npt.NDArray[np.float64]
            Numpy array of cones, each row is a cone and contains either
            [x, y] or [x, y, color, color_prob]

        Returns
        -------
        LandmarkArray
            Landmark array of cones
        """
        landmarks = []
        for cone in cones:
            landmark = Landmark()
            landmark.position.x = cone[0]
            landmark.position.y = cone[1]
            landmark.type = cone[2]
            landmarks.append(landmark)
        landmarksMsg = LandmarkArray()
        landmarksMsg.landmarks = landmarks
        return landmarksMsg

    def lidarCallback(self, cones: LandmarkArray) -> None:
        cones = self.toConeArray(cones)
        super().lidarCallback(cones)

    def smoreoCallback(self, cones: LandmarkArray) -> None:
        cones = self.toConeArray(cones, addType=True)
        super().smoreoCallback(cones)

    def run(self) -> LandmarkArray:
        """
        Runs the smornn algorithm and publishes the results
        """
        cones = super().run()
        if cones is None:
            return cones
        landmarks = self.toLandmarkArray(cones)
        self.publishers["detected"].publish(landmarks)
        detectedMarkers = self.markerViz.conesToMarkers(landmarks)
        self.publishers["detected_markers"].publish(detectedMarkers)
        return landmarks
