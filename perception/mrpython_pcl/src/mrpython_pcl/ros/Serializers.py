"""
Serializer functions for transforming between the following data types:
    - numpy array of points: npt.NDArray[np.float64]
    - pcl pointcloud: PointCloud
    - ros point cloud: sensor_msgs.msg.PointCloud2
"""
import rospy
import numpy as np
import numpy.typing as npt
from pcl import PointCloud, PointCloud_PointXYZI

from asurt_msgs.msg import Landmark, LandmarkArray
from sensor_msgs.msg import PointField, PointCloud2

# pylint: disable-msg=too-many-locals
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


def pclToRos(cloudPcl: PointCloud) -> PointCloud2:
    """
    Converts a pcl Point cloud to a ros PointCloud2 message

    Parameters
    ----------
    cloudPcl : PointCloud
        Point cloud to convert

    Returns
    -------
    PointCloud2
        Ready to publish ros PointCloud2 message
    """
    # Converting pcl to numpy array.
    # -NEVER DO THIS- cloud = np.asarray(cloudPcl)
    cloud = cloudPcl.to_array()

    return npToRos(cloud)


def npToRos(cloudArray: npt.NDArray[np.float64]) -> PointCloud2:
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

    # Fill the PointCLoud2 message definition
    # header
    try:
        cloudRos.header.stamp = rospy.Time.now()
    except rospy.exceptions.ROSInitException:
        rospy.logwarn("ROS not initialized, using time 0 for header")

    cloudRos.header.frame_id = "velodyne"

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


def npConesToRos(cones: npt.NDArray[np.float64], addIDs: bool = False) -> LandmarkArray:
    """
    Converts a list of cones to a LandmarkArray message

    Parameters
    ----------
    cones : npt.NDArray[np.float64]
        Locations of the cones
    addIDs : bool, by default False
        If true, the cone IDs are added to the message

    Returns
    -------
    LandmarkArray
        The created LandmarkArray message
    """
    landmarks = []

    for cone in cones:
        landmark = Landmark()
        landmark.position.x = cone[0]
        landmark.position.y = cone[1]
        landmark.type = 4  # Unknown type
        if addIDs:
            landmark.identifier = cone[2]
        landmarks.append(landmark)

    msg = LandmarkArray()
    msg.landmarks = landmarks
    msg.header.frame_id = "velodyne"
    try:
        msg.header.stamp = rospy.Time.now()
    except rospy.exceptions.ROSInitException:
        rospy.logwarn("ROS not initialized, using time 0 for header")

    return msg
