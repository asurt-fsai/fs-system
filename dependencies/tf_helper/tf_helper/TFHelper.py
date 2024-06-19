"""
Helper class to help with transforming messages between different tf frames
"""
from typing import Optional, Tuple, Any, Dict
import time
import numpy as np
import numpy.typing as npt
import transformations

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray


from utils import Utils


class TFHelper:
    """
    Helper class that can transform a message between any two frames available in tf
    """

    def __init__(self, nodeObject: Node):
        self.nodeObject = nodeObject
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self.nodeObject)
        time.sleep(0.2)  # To ensure a tf has been received
        self.warningPrinted: Dict[str, int] = {}
        self.utils = Utils(nodeObject)

    def getTransform(
        self, fromId: str, toId: str
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
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
            trans: TransformStamped = self.tfBuffer.lookup_transform(
                toId, fromId, self.nodeObject.get_clock().now().to_msg()
            )

            trans = trans.transform
            translation = trans.translation.x, trans.translation.y, trans.translation.z
            rot = transformations.euler_from_quaternion(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
            )
            return translation, rot

        except (LookupException, ConnectivityException, ExtrapolationException):
            if fromId + toId not in self.warningPrinted:
                self.nodeObject.get_logger().warn(
                    f"{self.nodeObject.get_name()}: Couldn't get tf from {fromId } to {toId}"
                )
                self.warningPrinted[fromId + toId] = 1
            else:
                self.warningPrinted[fromId + toId] += 1
                if self.warningPrinted[fromId + toId] > 10:
                    self.warningPrinted.pop(fromId + toId)
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def transformArr2d(
        self, arr: npt.NDArray[np.float64], fromId: str, toId: str
    ) -> Optional[npt.NDArray[np.float64]]:
        """
        Transform a given numpy array from a given frame to another

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
        trans, rot = transform
        sinYaw, cosYaw = np.sin(rot[2]), np.cos(rot[2])
        rotMat = np.array([[cosYaw, -sinYaw], [sinYaw, cosYaw]])

        arr = (rotMat @ arr.transpose()).transpose()
        arr += np.array([*trans[:2]]).reshape(1, 2)
        return arr

    def transformArr3d(
        self, arr: npt.NDArray[np.float64], fromId: str, toId: str
    ) -> Optional[npt.NDArray[np.float64]]:
        """
        Transform a given numpy array from a given frame to another

        Parameters
        ----------
        arr : npt.NDArray[np.float64]
            Array to transform, shape = (N, 3)
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
        trans, rot = transform
        cos1 = np.cos(rot[0])
        sin1 = np.sin(rot[0])
        cos2 = np.cos(rot[1])
        sin2 = np.sin(rot[1])
        cos3 = np.cos(rot[2])
        sin3 = np.sin(rot[2])
        rotMat = np.array(
            [
                [cos2 * cos3, -cos2 * sin3, sin2],
                [cos1 * sin3 + cos3 * sin1 * sin2, cos1 * cos3 - sin1 * sin2 * sin3, -cos2 * sin1],
                [sin1 * sin3 - cos1 * cos3 * sin2, cos3 * sin1 + cos1 * sin2 * sin3, cos1 * cos2],
            ]
        )

        arr = (rotMat @ arr.transpose()).transpose()
        arr += np.array([*trans]).reshape(1, 3)
        return arr

    def transformLandmarkArrayMsg(
        self, rosMsg: LandmarkArray, toId: str
    ) -> Optional[LandmarkArray]:
        """
        Transform a ros message of type LandmarkArray from its frame to another

        Parameters
        ----------
        rosMsg : LandmarkArray
            The ros message to transform
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
            fromId = rosMsg.header.frame_id
            transformedCones = self.transformArr2d(conesArr, fromId, toId)
            if transformedCones is None:
                return rosMsg

            # Modify original message
            for idx, landmark in enumerate(rosMsg.landmarks):
                landmark.position.x = transformedCones[idx][0]
                landmark.position.y = transformedCones[idx][1]

        rosMsg.header.frame_id = toId
        return rosMsg

    def transformMarkerArrayMsg(self, rosMsg: MarkerArray, toId: str) -> Optional[MarkerArray]:
        """
        Transform a ros message of type MarkerArray from its frame to another

        Parameters
        ----------
        rosMsg : MarkerArray
            The ros message to transform
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
            fromId = rosMsg.markers[0].header.frame_id
            transformedCones = self.transformArr2d(conesArr, fromId, toId)
            if transformedCones is None:
                return rosMsg

            # Modify original message
            for idx, marker in enumerate(rosMsg.markers):
                marker.pose.position.x = transformedCones[idx][0]
                marker.pose.posdxtion.y = transformedCones[idx][1]
                marker.header.frame_id = toId

        return rosMsg

    def transformPathMsg(self, rosMsg: Path, toId: str) -> Optional[Path]:
        """
        Transform a ros message of type Path from its frame to another

        Parameters
        ----------
        rosMsg : Path
            The ros message to transform
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
            fromId = rosMsg.header.frame_id
            transformedPoints = self.transformArr2d(pointsArr, fromId, toId)
            if transformedPoints is None:
                return rosMsg

            # Modify original message
            for idx, pose in enumerate(rosMsg.poses):
                pose.header.frame_id = toId
                pose.pose.position.x = transformedPoints[idx][0]
                pose.pose.position.y = transformedPoints[idx][1]

        rosMsg.header.frame_id = toId

        return rosMsg

    def transformPointcloud(self, rosMsg: PointCloud2, toId: str) -> Optional[PointCloud2]:
        """
        Transform a ros message of type PointCloud2 from its frame to another

        Parameters
        ----------
        rosMsg : PointCloud2
            The ros message to transform
        toId : str
            The frame to transform to

        Returns
        -------
        PointCloud2
            The transformed ros message
            If the transformation is not possible, the message is returned as is (with old frame_id)
        """
        fromId = rosMsg.header.frame_id
        npPc = self.utils.rosToPcl(rosMsg).to_array()
        npPc[:, :3] = self.transformArr3d(npPc[:, :3], fromId, toId)
        return self.utils.npToRos(npPc, toId)

    def transformMsg(self, rosMsg: Any, toId: str) -> Any:
        """
        Transform a ros message from a given frame to another

        Parameters
        ----------
        rosMsg : Any
            Ros message to transform
        toId : str
            Frame to transform to

        Returns
        -------
        Any or None
            The transformed ros message into the new frame
            Return the original message if the transformation is not possible
            Return None if the message type is not supported (also throws a rospy.logerr)
        """
        if isinstance(rosMsg, LandmarkArray):
            return self.transformLandmarkArrayMsg(rosMsg, toId)

        if isinstance(rosMsg, MarkerArray):
            return self.transformMarkerArrayMsg(rosMsg, toId)

        if isinstance(rosMsg, Path):
            return self.transformPathMsg(rosMsg, toId)

        if isinstance(rosMsg, PointCloud2):
            return self.transformPointcloud(rosMsg, toId)

        self.nodeObject.get_logger().error(f"{type(rosMsg)} not implemented in tf_helper")
        return None
