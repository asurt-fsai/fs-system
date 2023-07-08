"""
Module to filter the paths from planning
"""
from typing import Optional, List

import rospy
import numpy as np
import numpy.typing as npt
from scipy import interpolate
from nav_msgs.msg import Path
from tf_helper.TFHelper import TFHelper
from tf_helper.utils import parsePathMessage, createPathMessage


def fitSpline(waypoints: npt.NDArray[np.float64], nSamples: int = 20) -> npt.NDArray[np.float64]:
    """
    Fits a spline to the given waypoint list

    Parameters
    ----------
    waypoints: np.array, shape = [N,2]
        Waypoints to fit the spline to
    nSamples: int
        Number of points to sample from the spline

    Returns
    -------
    np.array, shape = (nSamples, 2)
        Points sampled from the fitted spline
    """
    if waypoints.shape[0] < 2:
        return waypoints

    degree = 2
    if waypoints.shape[0] == 2:
        degree = 1

    tck = interpolate.splprep(waypoints.T, w=np.ones(waypoints.shape[0]), s=20, k=degree)[0]
    unew = np.linspace(0, 1, nSamples)
    newWaypoints = np.array(interpolate.splev(unew, tck)).T
    return newWaypoints


class SimpleWaypointsCleaner:  # pylint: disable=too-many-instance-attributes
    """
    Simple waypoint cleaner to help filter the path from planning

    Parameters
    ----------
    cleanWaypointsTopic: str
        Topic to publish clean waypoints to
    pathTopic: str
        Topic to subscribe to in order to recieve the path from planning
    frameId: str
        Frame id to work in (usually should be map frame)
    nPathsToKeep: int
        Number of paths to keep in a row. Old paths will be discarded
    radiusToMergeWaypoints: float
        Radius to merge waypoints in meters
    minWaypointOccur: int
        Minimum number of times a waypoint should occur to be considered
    """

    def __init__(
        self,
        cleanWaypointsTopic: str,
        pathTopic: str,
        frameId: str,
        nPathsToKeep: int,
        radiusToMergeWaypoints: float,
        minWaypointOccur: int,
    ):
        self.helper = TFHelper("planning_simple_cleaner")
        self.frameId = frameId
        self.nPathsToKeep = nPathsToKeep
        self.radiusToMergeWaypoints = radiusToMergeWaypoints
        self.minWaypointOccur = minWaypointOccur
        self.maxWaypointJump = 5
        self.prevPaths: List[npt.NDArray[np.float64]] = []

        self.waypointsPub = rospy.Publisher(cleanWaypointsTopic, Path, queue_size=10)
        rospy.Subscriber(pathTopic, Path, self.addPath)

    def addPath(self, msg: Path) -> None:
        """
        Adds a path to the current list of paths

        Parameters
        ----------
        msg: Path
            The path message recieved from planning
        """
        msg = self.helper.transformMsg(msg, self.frameId)
        waypoints = parsePathMessage(msg)
        self.prevPaths.append(waypoints)
        self.prevPaths = self.prevPaths[-self.nPathsToKeep :]

    def getWaypoints(self) -> Optional[npt.NDArray[np.float64]]:
        """
        Filters the waypoints and fits a spline to them

        Returns
        -------
        None if no clean path found
        Otherwise, np.array, shape = (N,2). N is the number of waypoints returned
        """
        waypoints: List[npt.NDArray[np.float64]] = []
        for path in self.prevPaths:
            waypoints.extend(path)

        if len(waypoints) == 0:
            return None

        clusteredWaypointsArr = np.empty((0, 2)).astype(float)
        clusteredCount: List[int] = []
        for waypoint in waypoints:
            minDist = 9999
            minIdx = 0
            if clusteredWaypointsArr.shape[0] > 0:
                dists = np.linalg.norm(
                    np.subtract(waypoint.reshape(1, 2), clusteredWaypointsArr), axis=1
                )
                minIdx = np.argmin(dists).item()
                minDist = dists[minIdx]
            if minDist < self.radiusToMergeWaypoints:
                waypointCount = clusteredCount[minIdx]
                clusteredWaypointsArr[minIdx] = (
                    clusteredWaypointsArr[minIdx] * waypointCount + waypoint
                ) / (waypointCount + 1)
                clusteredCount[minIdx] += 1
            else:
                clusteredWaypointsArr = np.vstack((clusteredWaypointsArr, waypoint.reshape(1, 2)))
                clusteredCount.append(1)
        clusteredCountArr = np.array(clusteredCount)

        clusteredWaypointsArr = clusteredWaypointsArr[clusteredCountArr > self.minWaypointOccur]
        if clusteredWaypointsArr.shape[0] == 0:
            return None

        # get waypoints in order

        rearPosition = self.helper.getTransform("rear_link", self.frameId)[0][:2]
        orderedWaypoints = [np.array(rearPosition)]

        while clusteredWaypointsArr.shape[0] > 0:
            dists = np.linalg.norm(
                np.subtract(orderedWaypoints[-1].reshape(1, 2), clusteredWaypointsArr), axis=1
            )
            minIdx = np.argmin(dists).item()
            minDist = dists[minIdx]
            if len(orderedWaypoints) == 1 and minDist < 1:
                clusteredWaypointsArr = np.delete(clusteredWaypointsArr, minIdx, 0)
                continue

            if minDist > self.maxWaypointJump:
                break
            orderedWaypoints.append(clusteredWaypointsArr[minIdx])
            clusteredWaypointsArr = np.delete(clusteredWaypointsArr, minIdx, 0)
        if len(orderedWaypoints) == 1:
            return None
        smoothWaypoints = fitSpline(np.array(orderedWaypoints))
        return smoothWaypoints

    def run(self) -> None:
        """
        Fetches the cleaned waypoints and publishes them
        """
        waypoints = self.getWaypoints()
        if waypoints is None:
            return
        pathMsg = createPathMessage(waypoints, self.frameId)
        self.waypointsPub.publish(pathMsg)
