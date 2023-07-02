"""
This module calculates the centerline of a track
"""
from typing import Tuple
from centerline_calc import smooth
import rospy
import numpy.typing as npt
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped


class Centerline:
    """
    Class to calculate the centerline of a track
    inner: np.ndarray, shape=(n, 2)
        Inner track boundary array with only x and y coordinates
    outter: np.ndarray, shape=(n, 2)
        Outter track boundary array with only x and y coordinates
    filteredInner: np.ndarray, shape=(n, 2)
        Inner track boundary array after removing outliers
    filteredOutter: np.ndarray, shape=(n, 2)
        Outter track boundary array after removing outliers
    """

    def __init__(
        self,
        conesMessage: Path = None,
        outterArr: npt.NDArray[np.float64] = np.zeros((1, 2)),
        innerArr: npt.NDArray[np.float64] = np.zeros((1, 2)),
    ):
        self.outter: npt.NDArray[np.float64] = np.zeros((1, 2))
        self.inner: npt.NDArray[np.float64] = np.zeros((1, 2))
        if conesMessage is not None:
            self.outter, self.inner = self.conesCB(conesMessage)
        else:
            self.outter = outterArr
            self.inner = innerArr
        self.filteredInner = self.removeOutliers(innerArr)
        self.filteredOutter = self.removeOutliers(outterArr)
        print(self.filteredInner)
        self.smoothedInner = smooth.SmoothTrack(self.filteredInner)
        self.smoothedOutter = smooth.SmoothTrack(self.filteredOutter)
        self.centerline = self.calcCenter(self.smoothedOutter.path, self.smoothedInner.path)

    def removeOutliers(self, points: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Removes outliers from a track boundary array

        Parameters
        ----------
        points: np.ndarray, shape=(n, 2)
            Track boundary array with only x and y coordinates

        Returns
        -------
        filteredPoints: np.ndarray, shape=(n, 2)
            Track boundary array after removing outliers

        """
        for idx, point in enumerate(points):
            tempPoints = points
            tempPoints = np.delete(tempPoints, idx, axis=0)
            tempFilteredPoints = np.zeros((1, 2))
            tempFilteredPoints[0] = point
            while tempPoints.shape[0] > 0:
                distances = np.linalg.norm(tempPoints - tempFilteredPoints[-1], axis=1)
                minIndex = np.argmin(distances)
                if distances[minIndex] < 4:
                    tempFilteredPoints = np.append(
                        tempFilteredPoints, [tempPoints[minIndex]], axis=0
                    )
                    tempPoints = np.delete(tempPoints, minIndex, axis=0)
                else:
                    distanceFirst = np.linalg.norm(tempPoints - tempFilteredPoints[0], axis=1)
                    minIndexFirst = np.argmin(distanceFirst)
                    if distanceFirst[minIndexFirst] < 4:
                        tempFilteredPoints = np.insert(
                            tempFilteredPoints, 0, [tempPoints[minIndexFirst]], axis=0
                        )
                        tempPoints = np.delete(tempPoints, [minIndex, minIndexFirst], axis=0)
                    else:
                        tempPoints = np.delete(tempPoints, minIndex, axis=0)
            if tempFilteredPoints.shape[0] > points.shape[0] * 0.7:
                return tempFilteredPoints
        return points

    def calcCenter(
        self, outterPath: npt.NDArray[np.float64], innerPath: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """
        Calculates the centerline of a track
        Parameters
        ----------
        outterPath: np.ndarray, shape=(n, 2)
            Outter track boundary array with only x and y coordinates
        innerPath: np.ndarray, shape=(n, 2)
            Inner track boundary array with only x and y coordinates
        Returns
        -------
        centerline: np.ndarray, shape=(n, 2)
            Centerline of the track
        """
        centerline = np.zeros((1, 2))
        for point in outterPath:
            distances = np.linalg.norm(innerPath - point, axis=1)
            minIndex = np.argmin(distances)
            centerline = np.append(centerline, [(point + innerPath[minIndex]) / 2], axis=0)
        centerline = np.delete(centerline, 0, axis=0)
        return centerline

    def conesCB(self, msg: MarkerArray) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Callback function for the cones subscriber
        Parameters
        ----------
        msg: MarkerArray
            Message from the cones subscriber
        Returns
        -------
        blueCones: np.ndarray, shape=(n, 2)
            Array with only x and y coordinates of the blue cones
        yellowCones: np.ndarray, shape=(n, 2)
            Array with only x and y coordinates of the yellow cones
        """
        blueCones = np.zeros((1, 2))
        yellowCones = np.zeros((1, 2))
        for marker in msg.markers:
            conePose = marker.pose.position
            if marker.color.r == 0.0 and marker.color.g == 0.0 and marker.color.b == 1.0:
                blueCones = np.append(blueCones, [[conePose.x, conePose.y]], axis=0)
            elif marker.color.r == 1.0 and marker.color.g == 1.0 and marker.color.b == 0.0:
                yellowCones = np.append(yellowCones, [[conePose.x, conePose.y]], axis=0)
        blueCones = np.delete(blueCones, 0, axis=0)
        yellowCones = np.delete(yellowCones, 0, axis=0)
        return blueCones, yellowCones

    def centerlineToMsg(self, centerlineNP: npt.NDArray[np.float64]) -> Path:
        """
        Converts the centerline array to a Path message
        Parameters
        ----------
        centerlineNP: np.ndarray, shape=(n, 2)
            Centerline of the track
        Returns
        -------
        centerlineMsg: Path
            Path message of the centerline
        """
        centerlineMsg = Path()
        centerlineMsg.header.frame_id = "map"
        centerlineMsg.header.stamp = rospy.Time.now()
        for point in centerlineNP:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = centerlineMsg.header.stamp
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            centerlineMsg.poses.append(pose)
        return centerlineMsg
