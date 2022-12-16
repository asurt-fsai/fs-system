"""
Contains ground removal classes to be used with the Filter module.
Mainly contains the AdaptiveGroundRemoval class and the RansacGroundRemoval class.
"""
from dataclasses import dataclass

import pcl
import numpy as np
import numpy.typing as npt
from ..helpers import SingletonMeta

# To fix the issue with mypy not recognising the pcl classes
# These can be removed of course, they are just to make mypy happy
pcl.PointCloud = pcl.PointCloud
pcl.SampleConsensusModelPlane = pcl.SampleConsensusModelPlane
pcl.RandomSampleConsensus = pcl.RandomSampleConsensus


@dataclass
class GroundRemovalMethod:
    """
    Base class from ground removal methods
    """

    def removeGround(self, cloud: pcl.PointCloud) -> pcl.PointCloud:
        """
        Removes points falling onto the ground plance

        Parameters
        ----------
        cloud : pcl.PointCloud
            Point cloud containing the ground points and other
            points (the points that we are interested in)

        Returns
        -------
        pcl.PointCloud
            Point cloud with the the ground points removed
        """
        raise NotImplementedError


@dataclass
class AdaptiveGroundRemoval(GroundRemovalMethod, metaclass=SingletonMeta):  # type: ignore[misc]
    """
    Removes points falling onto the ground plance using an adaptive method
    Based on the paper: https://arxiv.org/abs/1905.05150
    """

    def __init__(self, nGridCellsX: int = 40, nGridCellsY: int = 40, distFromPlaneTh: float = 0.2):
        """
        Parameters
        ----------
        nGridCellsX : int, by default 40
            Number of grid cells in the x axis for the discretization
        nGridCellsY : int, by default 40
            Number of grid cells in the y axis for the discretization
        distFromPlaneTh : float, by default 0.2
            Maximum distance from the ground plane to be considered a ground point
        """
        self.nGridCells = [nGridCellsX, nGridCellsY]
        self.distFromPlaneTh = distFromPlaneTh

        # Parameter Validation
        try:
            assert nGridCellsX > 0 and int(nGridCellsX) == nGridCellsX
            assert nGridCellsY > 0 and int(nGridCellsY) == nGridCellsY
            assert distFromPlaneTh > 0 and type(distFromPlaneTh) in [int, float]
        except Exception as exp:
            errMsg = "AdaptiveGroundRemoval: ensure all parameters are within correct ranges \n \
                        nGridCellsX: int > 0 \n \
                        nGridCellsY: int > 0 \n \
                        distFromPlaneTh: float > 0"
            raise TypeError(errMsg) from exp

    def removeGround(self, cloud: pcl.PointCloud) -> pcl.PointCloud:
        """
        Removes points falling onto the ground plance

        Parameters
        ----------
        cloud : pcl.PointCloud
            Point cloud containing the ground points and other
            points (the points that we are interested in)

        Returns
        -------
        pcl.PointCloud
            Point cloud with the the ground points removed
        """
        points = cloud.to_array()

        locs = self.getDiscretizedLocs(points)  # Discretized locations for each point
        values = np.unique(locs)
        binMins = np.zeros((self.nGridCells[0] * self.nGridCells[1], 3))
        for val in values:
            pVals = points[locs == val]
            binMins[val] = pVals[np.argmin(pVals[:, 2])]

        # linear regression on binMins
        matX = np.hstack((binMins[:, :2], np.ones((binMins.shape[0], 1))))
        y = binMins[:, 2:]
        theta = np.linalg.pinv(matX) @ y

        inputs = np.hstack((points[:, :2], np.ones((points.shape[0], 1))))
        preds = inputs @ theta
        diffs = np.abs(points[:, 2] - preds.reshape(-1))
        points = points[diffs < self.distFromPlaneTh]

        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))
        return cloud

    def getDiscretizedLocs(self, points: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Discretizes the points into a grid of size nGridCells and returns the
        discretized locations for each point

        Parameters
        ----------
        points: np.ndarray
            Points to discretize
        nGridCells: Tuple[int, int]
            Number of grid cells in the x and y axes

        Returns
        -------
        locs: np.ndarray
            Discretized locations for each point
        """
        xLimits = [points[:, 0].min(), points[:, 0].max()]
        yLimits = [points[:, 1].min(), points[:, 1].max()]
        limits = {"x": xLimits, "y": yLimits}
        locs = np.copy(points)
        locs[:, 0] -= limits["x"][0]
        locs[:, 1] -= limits["y"][0]
        locs[:, 0] /= limits["x"][1] - limits["x"][0]
        locs[:, 1] /= limits["y"][1] - limits["y"][0]
        locs[:, 0] *= self.nGridCells[0] - 1
        locs[:, 1] *= self.nGridCells[1] - 1
        locs = locs.astype(np.int32)
        locs = locs[:, 0] * self.nGridCells[1] + locs[:, 1]
        return locs


@dataclass
class RansacGroundRemoval(GroundRemovalMethod, metaclass=SingletonMeta):  # type: ignore[misc]
    """
    Removes points falling onto the ground plance using RANSAC implemented in python-pcl
    """

    def __init__(self, ransacTh: float):
        """
        Parameters
        ----------
        ransacTh : float
            The ransac threshold used to determine if a point is part of the ground plane
        """
        self.ransacTh = ransacTh

        # Parameter Validation
        try:
            assert ransacTh > 0 and type(ransacTh) in [int, float]
        except Exception as exp:
            errMsg = "RansacGroundRemoval: ensure all parameters are within correct ranges \n \
                        ransacTh: float > 0"
            raise TypeError(errMsg) from exp

    def removeGround(self, cloud: pcl.PointCloud) -> pcl.PointCloud:
        """
        Removes points falling onto the ground plance

        Parameters
        ----------
        cloud : pcl.PointCloud
            Point cloud containing the ground points and other
            points (the points that we are interested in)

        Returns
        -------
        pcl.PointCloud
            Point cloud with the the ground points removed
        """
        # Model for 3D plane segmentation
        planeModel = pcl.SampleConsensusModelPlane(cloud)

        # Run RANSAC
        ransac = pcl.RandomSampleConsensus(planeModel)
        ransac.set_DistanceThreshold(self.ransacTh)
        ransac.computeModel()
        inliers = ransac.get_Inliers()

        # Remove ground points
        cloudFiltered = cloud.extract(inliers, True)

        return cloudFiltered
