"""
General helper functions for all packages
"""
from typing import Union, List

import rospy
import numpy as np
import numpy.typing as npt
from nav_msgs.msg import Path
from std_msgs.msg import Header
from asurt_msgs.msg import Landmark, LandmarkArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


def parseLandmarks(landmarks: LandmarkArray) -> npt.NDArray[np.float64]:
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
    cones: npt.NDArray[np.float64],
    types: npt.NDArray[np.int8],
    coneProbs: npt.NDArray[np.float64],
    frameId: str,
    timestamp: rospy.Time = None,
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
        msg.header.stamp = rospy.Time.now()
    else:
        msg.header.stamp = timestamp
    return msg


def createPathMessage(
    waypoints: Union[List[float], npt.NDArray[np.float64]],
    frameId: str,
    timestamp: rospy.Time = None,
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
        outputPath.header.stamp = rospy.Time.now()
    else:
        outputPath.header.stamp = timestamp

    if waypoints.shape[0] > 0:
        outputPath.poses = [
            PoseStamped(
                Header(frameId=frameId), Pose(Point(x=waypoint[0], y=waypoint[1]), Quaternion())
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
