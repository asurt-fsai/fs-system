"""
Adaptive Ground Removal based on the paper: https://arxiv.org/abs/1905.05150
"""
from dataclasses import dataclass

from pcl import PointCloud
import numpy as np
import numpy.typing as npt

from ..helpers import SingletonMeta
from .GroundRemovalMethod import GroundRemovalMethod


@dataclass
class AdaptiveGroundRemoval(GroundRemovalMethod, metaclass=SingletonMeta):
    """
    Removes points falling onto the ground plane using an adaptive method
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

    def removeGround(self, cloud: PointCloud) -> PointCloud:
        """
        Removes points falling onto the ground plane

        Parameters
        ----------
        cloud : PointCloud
            Point cloud containing the ground points and other
            points (the points that we are interested in)

        Returns
        -------
        PointCloud
            Point cloud with the the ground points removed
        """
        points = cloud.to_array()

        locs = self.getDiscretizedLocs(points)  # Discretized locations for each point
        values = np.unique(locs)
        binMins = np.zeros((self.nGridCells[0] * self.nGridCells[1], 3))
        binMinsMask = np.zeros((self.nGridCells[0] * self.nGridCells[1]))
        for val in values:
            pVals = points[locs == val]
            binMins[val] = pVals[np.argmin(pVals[:, 2])]
            binMinsMask[val] = 1

        # linear regression on binMins
        binMins = binMins[binMinsMask == 1]
        matX = np.hstack((binMins[:, :2], np.ones((binMins.shape[0], 1))))
        y = binMins[:, 2:]
        theta = np.linalg.pinv(matX) @ y

        inputs = np.hstack((points[:, :2], np.ones((points.shape[0], 1))))
        preds = inputs @ theta
        diffs = np.abs(points[:, 2] - preds.reshape(-1))
        points = points[diffs > self.distFromPlaneTh]

        cloud = PointCloud()
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
