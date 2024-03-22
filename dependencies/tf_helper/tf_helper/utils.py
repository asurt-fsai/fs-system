"""
General helper functions for all packages
"""
from typing import Union, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
import numpy.typing as npt
from nav_msgs.msg import Path
from std_msgs.msg import Header
from pcl import PointCloud, PointCloud_PointXYZI
from sensor_msgs.msg import PointField, PointCloud2
from asurt_msgs.msg import Landmark, LandmarkArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class Utils:
    def __init__(self, node: Node):
        self.node = node

    def parseLandmarks(self, landmarks: LandmarkArray) -> npt.NDArray[np.float64]:
        """
        Parse a given landmark array of cones into a numpy array

        Parameters
        ----------
        landmarks : LandmarkArray
            Ros message to parse

        Returns
        -------
        npt.NDArray[np.float64]
            Parsed numpy array, each row contains [pos x, pos y, cone type, color probability]
        """
        cones = []
        for landmark in landmarks.landmarks:
            cones.append(
                [landmark.position.x, landmark.position.y, landmark.type, landmark.probability]
            )
        return np.array(cones)


    def createLandmarkMessage(
        self,
        cones: npt.NDArray[np.float64],
        types: npt.NDArray[np.int8],
        coneProbs: npt.NDArray[np.float64],
        frameId: str,
        timestamp: Time = None,
    ) -> LandmarkArray:
        """
        Construct a LandmarkArray message using cone positions and types
        Note: No way to add the cone ids

        Parameters
        ----------
        cones : npt.NDArray[np.float64]
            Cone positions, each row contains [pos x, pos y]
        types : npt.NDArray[np.int8]
            Cone types according to the standard in the Landmark message
        coneProbs : npt.NDArray[np.float64]
            For each cone, the probability the color of it is correct
        frameId : str
            Frame the message will be in
        timestamp : rospy.Time, optional
            Override the current time by this timestamp

        Returns
        -------
        LandmarkArray
            Constructs LandmarkArray message
        """
        landmarks = []

        for cone, typ, prob in zip(cones, types, coneProbs):
            coneMsg = Landmark()
            coneMsg.position.x = cone[0]
            coneMsg.position.y = cone[1]
            coneMsg.type = int(typ)
            coneMsg.probability = prob
            landmarks.append(coneMsg)

        msg = LandmarkArray()
        msg.landmarks = landmarks
        msg.header.frame_id = frameId
        if timestamp is None:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        else:
            msg.header.stamp = timestamp.to_msg()
        return msg

    def createPathMessage(
        self,
        waypoints: Union[List[float], npt.NDArray[np.float64]],
        frameId: str,
        timestamp: Time = None,
    ) -> Path:
        """
        Create a path message from a list of waypoints

        Parameters
        ----------
        waypoints : List or numpy array of points
            Each row is a waypoint [x, y]
        frameId : str
            Frame the message will be in
        timestamp : rospy.Time, optional
            Override the current time by this timestamp

        Returns
        -------
        Path
            Path message created from the waypoints
        """
        if isinstance(waypoints, list):
            waypoints = np.array(waypoints)

        outputPath = Path()
        outputPath.header.frame_id = frameId
        if timestamp is None:
            outputPath.header.stamp = self.node.get_clock().now().to_msg()
        else:
            outputPath.header.stamp = timestamp.to_msg()

        if waypoints.shape[0] > 0:
            outputPath.poses = [
                PoseStamped(
                    Header(frame_id=frameId), Pose(Point(x=waypoint[0], y=waypoint[1]), Quaternion())
                )
                for waypoint in waypoints
            ]
        return outputPath


    def parsePathMessage(path: Path) -> npt.NDArray[np.float64]:
        """
        Parse a path message into a numpy array

        Parameters
        ----------
        path : Path
            Path message to parse

        Returns
        -------
        npt.NDArray[np.float64]
            Parsed numpy array, each row contains [pos x, pos y]
        """
        waypoints = []
        for pose in path.poses:
            waypoints.append([pose.pose.position.x, pose.pose.position.y])
        return np.array(waypoints)


    def npToPcl(cloudArray: npt.NDArray[np.float64]) -> PointCloud:
        """
        Converts a numpy array of points to a pcl point cloud

        Parameters
        ----------
        cloudArray: np.array
            Point cloud array, each row contains [x,y,z] of a point

        Returns
        -------
        PointCloud
            Point cloud containing the same points as cloudArray
        """
        ###########################################################################
        ########################   This is slow  ##################################
        ###########################################################################
        if cloudArray.shape[1] == 3:
            cloud = PointCloud()
        elif cloudArray.shape[1] == 4:
            cloud = PointCloud_PointXYZI()

        cloud.from_array(cloudArray.astype(np.float32))

        return cloud


    # pylint: disable=too-many-locals
    def rosToPcl(rosPc2: PointCloud2, squeeze: bool = True) -> PointCloud:
        """
        Parses a given PointCloud2 ros message into a pcl point cloud

        Parameters
        ----------
        rosPc2: sensor_msgs.msg.PointCloud2
            Ros message to parse

        Returns
        -------
        cloud: PointCloud
            Parsed point cloud
        """
        dummyFieldPrefix = "__"
        typeMappings = [
            (PointField.INT8, np.dtype("int8")),
            (PointField.UINT8, np.dtype("uint8")),
            (PointField.INT16, np.dtype("int16")),
            (PointField.UINT16, np.dtype("uint16")),
            (PointField.INT32, np.dtype("int32")),
            (PointField.UINT32, np.dtype("uint32")),
            (PointField.FLOAT32, np.dtype("float32")),
            (PointField.FLOAT64, np.dtype("float64")),
        ]
        pftypeSizes = {
            PointField.INT8: 1,
            PointField.UINT8: 1,
            PointField.INT16: 2,
            PointField.UINT16: 2,
            PointField.INT32: 4,
            PointField.UINT32: 4,
            PointField.FLOAT32: 4,
            PointField.FLOAT64: 8,
        }
        pftypeToNptype = dict(typeMappings)

        offset = 0
        npDtypeList = []

        for field in rosPc2.fields:
            while offset < field.offset:
                # might be extra padding between fields
                npDtypeList.append((f"{dummyFieldPrefix}{offset}", np.uint8))
                offset += 1

            dtype = pftypeToNptype[field.datatype]
            if field.count != 1:
                dtype = np.dtype((dtype, field.count))

            npDtypeList.append((field.name, dtype))
            offset += pftypeSizes[field.datatype] * field.count

        # might be extra padding between points
        while offset < rosPc2.point_step:
            npDtypeList.append((f"{dummyFieldPrefix}{offset}", np.uint8))
            offset += 1

        # construct a numpy record type equivalent to the point type of this cloud
        dtypeList = npDtypeList

        # parse the cloud into an array
        if len(rosPc2.data) == 0:
            return PointCloud()

        cloudArr = np.fromstring(rosPc2.data, dtypeList)  # type: ignore[call-overload]

        # remove the dummy fields that were added
        dummyIdx = []
        for fname, _ in dtypeList:
            if not fname[: len(dummyFieldPrefix)] == dummyFieldPrefix:
                dummyIdx.append(fname)
        cloudArr = cloudArr[dummyIdx]

        nparray = None
        if squeeze and rosPc2.height == 1:
            nparray = np.reshape(cloudArr, (rosPc2.width,))
        else:
            nparray = np.reshape(cloudArr, (rosPc2.height, rosPc2.width))

        x = nparray["x"]
        y = nparray["y"]
        z = nparray["z"]
        if nparray.dtype.names is not None and "intensity" in nparray.dtype.names:
            intensity = nparray["intensity"]
            cloudNp = np.array([x, y, z, intensity]).T
        else:
            cloudNp = np.array([x, y, z]).T

        cloud = npToPcl(cloudNp)
        return cloud


    def npToRos(self, cloudArray: npt.NDArray[np.float64], frameId: str) -> PointCloud2:
        """
        Converts a numpy array of points to a ros PointCloud2 message

        Parameters
        ----------
        cloudArray : np.array
            Numpy array of points to convert

        Returns
        -------
        PointCloud2
            Ready to publish ros PointCloud2 message
        """
        cloudArray = cloudArray.astype(np.float32)

        # Instantiating a PointCloud2 object.
        cloudRos = PointCloud2()

        # Fill the PointCloud2 message definition
        cloudRos.header.stamp = self.node.get_clock().now().to_msg()

        cloudRos.header.frame_id = frameId

        # width & height
        points, dim = cloudArray.shape
        cloudRos.width = points
        cloudRos.height = 1

        # fields
        cloudRos.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        cloudRos.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        cloudRos.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        if dim == 4:
            cloudRos.fields.append(
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
            )

        cloudRos.is_dense = False
        cloudRos.is_bigendian = False

        # steps

        # itemsize tells us the size of an element
        # multiply by 4 because each point has 4 elements
        pointSize = cloudArray.itemsize * dim  # ps is 16

        cloudRos.point_step = pointSize
        cloudRos.row_step = points * pointSize

        # data
        cloudRos.data = cloudArray.tobytes()

        return cloudRos
