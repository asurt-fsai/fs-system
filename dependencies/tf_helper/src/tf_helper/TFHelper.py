"""
Helper class to help with transforming messages between different tf frames
"""
from typing import Optional, Tuple, Any
import tf
import tf2_ros
import rospy

import numpy as np
import numpy.typing as npt

from nav_msgs.msg import Path
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray


class TFHelper:
    """
    Helper class that can transform a message between any two frames available in tf
    """

    def __init__(self, nodeName: str):
        self.nodeName = nodeName
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(0.2)  # To ensure a tf has been received

    def getTransform(self, fromId: str, toId: str) -> Optional[Tuple[Tuple[float, float], float]]:
        """
        Fetches the transform (translation and rotation around z) if avaiable between the
        two given frames

        Parameters
        ----------
        fromId : str
            Frame to transform from
        toId : str
            Frame to transform to

        Returns
        -------
        Tuple[Tuple[float, float], float] or None
            If the transformation is not found, return None
            Else return a tuple (translation, yaw)
            translation: tuple of two float (translation in x, translation in y)
            yaw: float, rotation around z axis
        """
        try:
            trans = self.tfBuffer.lookup_transform(toId, fromId, rospy.Time(0))
            trans = trans.transform
            translation = trans.translation.x, trans.translation.y
            yaw = tf.transformations.euler_from_quaternion(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
            )[2]
            return translation, yaw
        # pylint: disable=no-member
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn(self.nodeName + ": Couldn't get tf from " + fromId + " to " + toId)
            return None

    def transformArr(
        self, arr: npt.NDArray[np.float64], fromId: str, toId: str
    ) -> Optional[npt.NDArray[np.float64]]:
        """
        Transfomr a given numpy array from a given frame to another

        Parameters
        ----------
        arr : npt.NDArray[np.float64]
            Array to transform, shape = (N, 2)
        fromId : str
            Frame to transform from
        toId : str
            Frame to transform to

        Returns
        -------
        npt.NDArray[np.float64] or None
            Transformed array
            Returns None if no transformation is possible between the given frames
        """
        transform = self.getTransform(fromId, toId)
        if transform is None:
            return None
        trans, yaw = transform
        sinYaw, cosYaw = np.sin(yaw), np.cos(yaw)
        rotMat = np.array([[cosYaw, -sinYaw], [sinYaw, cosYaw]])

        arr = (rotMat @ arr.transpose()).transpose()
        arr += np.array([*trans]).reshape(1, 2)
        return arr

    def getMessageIn(self, rosMsg: Any, toId: str) -> Any:
        """
        Transforms a given message to a new frame

        Parameters
        ----------
        rosMsg : Any
            Ros message to transform
        toId : str
            Frame to transform to

        Returns
        -------
        Any
            The ros message transformed to the new frame
        """
        if isinstance(rosMsg, LandmarkArray):
            fromId = rosMsg.header.frame_id
            return self.transformMsg(rosMsg, fromId, toId)
        if isinstance(rosMsg, MarkerArray):
            fromId = rosMsg.markers[0].header.frame_id
            return self.transformMsg(rosMsg, fromId, toId)
        errMsg = f"Message of type {type(rosMsg)} hasn't been implemented by TFHelper"
        raise NotImplementedError(errMsg)

    def transformLandmarkArrayMsg(
        self, rosMsg: LandmarkArray, fromId: str, toId: str
    ) -> Optional[LandmarkArray]:
        """
        Transform a ros message of type LandmarkArray from a frame to anoterh

        Parameters
        ----------
        rosMsg : LandmarkArray
            The ros message to transform
        fromId : str
            The frame to transform from
        toId : str
            The frame to transform to

        Returns
        -------
        LandmarkArray
            The transformed ros message
            If the transformation is not possible, the message is returned as is (with old frame_id)
        """
        # Extract cone locations
        cones = []
        for landmark in rosMsg.landmarks:
            cones.append([landmark.position.x, landmark.position.y])
        conesArr = np.array(cones)

        # Apply transformations
        if len(conesArr) > 0:
            transformedCones = self.transformArr(conesArr, fromId, toId)
            if transformedCones is None:
                return None

            # Modify original message
            for idx, landmark in enumerate(rosMsg.landmarks):
                landmark.position.x = transformedCones[idx][0]
                landmark.position.y = transformedCones[idx][1]

        rosMsg.header.frame_id = toId
        return rosMsg

    def transformMarkerArrayMsg(
        self, rosMsg: MarkerArray, fromId: str, toId: str
    ) -> Optional[MarkerArray]:
        """
        Transform a ros message of type MarkerArray from a frame to anoterh

        Parameters
        ----------
        rosMsg : MarkerArray
            The ros message to transform
        fromId : str
            The frame to transform from
        toId : str
            The frame to transform to

        Returns
        -------
        MarkerArray
            The transformed ros message
            If the transformation is not possible, the message is returned as is (with old frame_id)
        """
        # Extract cone locations
        cones = []
        for marker in rosMsg.markers:
            cones.append([marker.pose.position.x, marker.pose.position.y])
        conesArr = np.array(cones)

        # Apply transformations
        if len(conesArr) > 0:
            transformedCones = self.transformArr(conesArr, fromId, toId)
            if transformedCones is None:
                return None

            # Modify original message
            for idx, marker in enumerate(rosMsg.markers):
                marker.pose.position.x = transformedCones[idx][0]
                marker.pose.posdxtion.y = transformedCones[idx][1]
                marker.header.frame_id = toId

        return rosMsg

    def transformPathMsg(self, rosMsg: Path, fromId: str, toId: str) -> Optional[Path]:
        """
        Transform a ros message of type Path from a frame to anoterh

        Parameters
        ----------
        rosMsg : Path
            The ros message to transform
        fromId : str
            The frame to transform from
        toId : str
            The frame to transform to

        Returns
        -------
        Path
            The transformed ros message
            If the transformation is not possible, the message is returned as is (with old frame_id)
        """
        points = []
        for pose in rosMsg.poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
        pointsArr = np.array(points)

        # Apply transformations
        if len(pointsArr) > 0:
            transformedPoints = self.transformArr(pointsArr, fromId, toId)
            if transformedPoints is None:
                return None

            # Modify original message
            for idx, pose in enumerate(rosMsg.poses):
                pose.header.frame_id = toId
                pose.pose.position.x = transformedPoints[idx][0]
                pose.pose.position.y = transformedPoints[idx][1]

        rosMsg.header.frame_id = toId

        return rosMsg

    def transformMsg(self, rosMsg: Any, fromId: str, toId: str) -> Any:
        """
        Transform a ros message from a given frame to another

        Parameters
        ----------
        rosMsg : Any
            Ros message to transform
        fromId : str
            Frame to transform from
        toId : str
            Frame to transform to

        Returns
        -------
        Any or None
            The transformed ros message into the new frame
            Return None if the transformation is not possible either because:
                - No transformation found between the two given frames
                - The message type is not support (also throws a rospy.logerr)
        """
        if isinstance(rosMsg, LandmarkArray):
            return self.transformLandmarkArrayMsg(rosMsg, fromId, toId)

        if isinstance(rosMsg, MarkerArray):
            return self.transformMarkerArrayMsg(rosMsg, fromId, toId)

        if isinstance(rosMsg, Path):
            return self.transformPathMsg(rosMsg, fromId, toId)

        rospy.logerr(type(rosMsg), " not implemented in tf_helper")
        return None
