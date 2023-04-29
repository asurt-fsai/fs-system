"""
This module contains the dataclasses for the track.
"""
from typing import Tuple
import rospy
import numpy as np
import numpy.typing as npt
from nav_msgs.msg import Path
from lqr import SmoothTrack, OptimizedTrack, SolverMatrices

TRACK_WIDTH: float = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN: float = rospy.get_param("/navigation/lqr/handler/safety_margin")


class Track:
    """
    This class conains all the track stages
    original: The original track data
    smooth: The smoothed track data
    optimized: The optimized track data
    final: The final track data
    """

    def __init__(self, receivedPathMsg: Path):
        self.receivedPath = self.pathToNumpy(receivedPathMsg)
        self.path = self.addWidth(self.receivedPath)
        self.smooth, self.solveMatrices, self.optimized = self.handler()

    def pathToNumpy(self, path: Path) -> npt.NDArray[np.float64]:
        """
        Converts a path message to a numpy array
        and transforms the coordinates to the world frame

        Parameters
        ----------
        path: Path
            Path message

        Returns
        -------
        pathArr: np.array, shape=(N,2)
            Path points in the world frame.
        """
        pathArr: npt.NDArray[np.float64] = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses]
        )
        return pathArr

    def addWidth(self, path: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Adds width of track to path array

        Parameters
        ----------
        path: np.ndarray, shape=(n, 2)
            Track array with only x and y coordinates

        Returns
        -------
        wideTrack: np.ndarray, shape=(n, 4)
            Track array with x, y, left width and right width
        """

        widthCol = np.ones(int(path.shape[0])) * (TRACK_WIDTH - SAFETY_MARGIN)
        path = np.stack(
            (
                path[:, 0],
                path[:, 1],
                widthCol / 2,
                widthCol / 2,
            ),
            axis=1,
        )
        wideTrack = np.reshape(path, (-1, 4))
        return wideTrack

    def handler(self) -> Tuple[SmoothTrack, SolverMatrices, OptimizedTrack]:
        """
        This function go through all the track processing stages
        """
        smooth = SmoothTrack(self.path)
        solveMatrices = SolverMatrices(smooth)
        optimized = OptimizedTrack(smooth)

        return smooth, solveMatrices, optimized
