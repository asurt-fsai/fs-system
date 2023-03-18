"""
Implementation of Smornn:
A class that combines the detections from the lidar and smoreo pipelines
"""
from typing import Optional
from threading import Lock

import numpy as np
import numpy.typing as npt

from .helpers import mutexLock


class Smornn:
    """
    Main class for Smornn
    Uses both the detections from the lidar and smoreo pipelines and combines them
    The closest detection from smoreo to a detection from the lidar is used to color
    the lidar detections. In other words: Position is used from the lidar and color
    is used from smoreo

    Parameters
    ----------
    minDistNeighbor: float
        The distance where (less than it) a smoreo detection and a lidar detection
         are considered the same cone
    """

    mutex = (
        Lock()
    )  # Used to prevent race conditions when settings and readings lidar and smoreo data

    def __init__(self, minDistNeighbor: float):
        self.minDistNeighbor = minDistNeighbor
        self.lidarCones: Optional[npt.NDArray[np.float64]] = None
        self.smoreoCones: Optional[npt.NDArray[np.float64]] = None

    @mutexLock(mutex)
    def lidarCallback(self, cones: npt.NDArray[np.float64]) -> None:
        """
        Used to get a lidar detection reading

        Parameters
        ----------
        cones: np.array, shape=[N, 2]
            The cones detection, N is the number of cones, each row contains
                [cone_x, cone_y]
        """
        self.lidarCones = cones

    @mutexLock(mutex)
    def smoreoCallback(self, cones: npt.NDArray[np.float64]) -> None:
        """
        Used to get a smoreo detection reading

        Parameters
        ----------
        cones: np.array, shape=[N, 4]
            The cones detection, N is the number of cones, each row contains
                [cone_x, cone_y, cone_color, cone_color_prob]
        """
        self.smoreoCones = cones[~np.isnan(cones).any(axis=1)]  # Remove nans

    @mutexLock(mutex)
    def run(self) -> Optional[npt.NDArray[np.float64]]:
        """
        Run smornn on the available lidar and smoreo readings
        Returns None if no lidar detections are available
        Returns lidar detections with all cone_color == 4 (unknown) if no smoreo readings available
        Otherwise, does the nearest neighbors to get colors for the lidar detections

        Returns
        -------
        None if no lidar detections are available
        or
        coloredCones: np.array, shape=[N, 4]
            The cones detection, N is the number of cones, each row contains
                [cone_x, cone_y, cone_color, cone_color_prob]
            If no smoreo detections are available, cone_color is set to 4 (unknown)
        """
        if self.lidarCones is None or len(self.lidarCones) == 0:
            return None

        # If smoreo isn't working, return lidar with unknown classes
        if self.smoreoCones is None or len(self.smoreoCones) == 0:
            return self.constructConesWithColors(self.lidarCones)

        smoreoCones = self.smoreoCones[:, :2].reshape(1, -1, 2)
        lidarCones = self.lidarCones.reshape(-1, 1, 2)

        diffs = np.linalg.norm(smoreoCones - lidarCones, axis=2)  # shape=(N_lidar, N_smoreo)
        bestCones = np.argmin(diffs, axis=1)
        mins = np.min(diffs, axis=1)

        # Add colors
        coneColors = self.smoreoCones[:, 2][bestCones]
        coneProbs = self.smoreoCones[:, 3][bestCones]

        toLargeDistIdx = mins > self.minDistNeighbor
        coneColors[toLargeDistIdx] = 4  # Unknown type
        coneProbs[toLargeDistIdx] = 1  # Unknown type, prob = 1

        coloredCones = self.constructConesWithColors(self.lidarCones, coneColors, coneProbs)

        # Remove detections to prevent repeating the same message
        self.lidarCones = None
        self.smoreoCones = None

        return coloredCones

    def constructConesWithColors(
        self,
        cones: npt.NDArray[np.float64],
        colors: Optional[npt.NDArray[np.float64]] = None,
        probs: Optional[npt.NDArray[np.float64]] = None,
    ) -> npt.NDArray[np.float64]:
        """
        Create an array of cones with colors and probabilities

        Parameters
        ----------
        cones : npt.NDArray[np.float64]
            Positions of the cones, shape=(N, 2)
            Each row contains [cone_x, cone_y]
        colors : npt.NDArray[np.float64], by default None
            Colors of the cones, shape=(N)
        probs : npt.NDArray[np.float64], by default None
            Probabilities of the colors, shape=(N)
            Each element is the probability the given color is correct

        Returns
        -------
        npt.NDArray[np.float64]
            Array of cones with colors and probabilities, shape=(N, 4)
            Each row contains [cone_x, cone_y, cone_color, cone_color_prob]
        """
        if colors is None or probs is None:
            colors = np.ones(len(cones)) * 4  # Unknown type
            probs = np.ones(len(cones))
        coloredCones = np.zeros((len(cones), 4))
        coloredCones[:, :2] = cones
        coloredCones[:, 2] = colors
        coloredCones[:, 3] = probs
        return coloredCones
