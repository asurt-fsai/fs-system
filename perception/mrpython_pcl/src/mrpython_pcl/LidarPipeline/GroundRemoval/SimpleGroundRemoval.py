"""
Simple ground removal method
"""
from typing import List

from pcl import PointCloud
import numpy as np
import numpy.typing as npt
from ..helpers import SingletonMeta
from .GroundRemovalMethod import GroundRemovalMethod


class SimpleGroundRemoval(GroundRemovalMethod, metaclass=SingletonMeta):
    """
    Custom implementation to remove the ground plane

    Parameters
    ----------
    startingValues : List[float]
        Starting values for the plane equation
    """

    def __init__(self, startingValues: List[float], groundTh: float, nIters: int = 30):
        self.startingValues = startingValues
        self.groundTh = groundTh
        self.nIters = nIters

    def removeGround(self, cloud: PointCloud) -> PointCloud:
        """
        Fit a plane to a point cloud given a starting plane equation and
        constraints on the possible planes
        Used to detect the ground plane and remove the ground points

        Parameters
        ----------
        cloud : PointCloud
            Points to fit the plane to

        Returns
        -------
        PointCloud
            Point cloud with the ground points removed
        """
        points = cloud.to_array()

        planeEquation = self.startingValues

        for _ in range(self.nIters):
            neighbors = self.getNeighbors(points, planeEquation, subsample=0.2)
            planeEquation = self.fitPlane(neighbors)
        outliers = self.getOutliers(points, planeEquation)

        filteredCloud = PointCloud()
        filteredCloud.from_array(outliers.astype(np.float32))

        return filteredCloud

    def getNeighbors(
        self, points: npt.NDArray[np.float64], planeEquation: List[float], subsample: float
    ) -> npt.NDArray[np.float64]:
        """
        Get the points that are within a certain distance from the plane

        Parameters
        ----------
        points : npt.NDArray[np.float64]
            Points to get the neighbors of
        planeEquation : List[float]
            Plane equation to get the neighbors of (ax + by + cz + d = 0)
            This list contains [a, b, c, d]

        Returns
        -------
        npt.NDArray[np.float64]
            Points that are within a certain distance from the plane
        """
        planeA, planeB, planeC, planeD = planeEquation
        nPoints = points.shape[0]
        sampleIdx = np.random.choice(nPoints, int(nPoints * subsample), replace=False)
        pointsSample = points[sampleIdx, :]

        distances = (
            planeA * pointsSample[:, 0]
            + planeB * pointsSample[:, 1]
            + planeC * pointsSample[:, 2]
            + planeD
        ) / np.sqrt(planeA**2 + planeB**2 + planeC**2)

        neighborsIdx = np.where(np.abs(distances) <= self.groundTh)[0]
        toReturn: npt.NDArray[np.float64] = pointsSample[neighborsIdx]
        return toReturn

    def fitPlane(self, points: npt.NDArray[np.float64]) -> List[float]:
        """
        Fit a plane to a set of points

        Parameters
        ----------
        points : npt.NDArray[np.float64]
            Points to fit the plane to

        Returns
        -------
        List[float]
            Plane equation (ax + by + cz + d = 0)
            The list contains [a, b, c, d]
        """
        eqX = np.hstack((points[:, 0:2], np.ones((points.shape[0], 1))))
        planeA, planeB, planeD = np.linalg.lstsq(eqX, points[:, 2], rcond=None)[0]
        toReturn = [planeA, planeB, -1, planeD]
        return toReturn

    def getOutliers(
        self, points: npt.NDArray[np.float64], planeEquation: List[float]
    ) -> npt.NDArray[np.float64]:
        """
        Get the points that are outside a certain distance from the plane

        Parameters
        ----------
        points : npt.NDArray[np.float64]
            Points to get the outliers of
        planeEquation : List[float]
            Plane equation to get the outliers of (ax + by + cz + d = 0)
            The list contains [a, b, c, d]

        Returns
        -------
        npt.NDArray[np.float64]
            Points that are outside a certain distance from the plane
        """
        planeA, planeB, planeC, planeD = planeEquation

        distances = (
            planeA * points[:, 0] + planeB * points[:, 1] + planeC * points[:, 2] + planeD
        ) / np.sqrt(planeA**2 + planeB**2 + planeC**2)

        outliersIdx = np.where(np.abs(distances) > self.groundTh)[0]
        toReturn: npt.NDArray[np.float64] = points[outliersIdx]
        return toReturn
