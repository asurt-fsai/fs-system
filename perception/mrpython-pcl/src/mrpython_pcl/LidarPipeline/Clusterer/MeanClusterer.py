"""
Implementation of the Mean Shift Clustering algorithm from the LaserNet paper
"""
from typing import List, Tuple, Any, Dict

import numpy as np
import numpy.typing as npt

from ..helpers import SingletonMeta
from .AbstractClusterer import Clusterer


class MeanClusterer(Clusterer, metaclass=SingletonMeta):
    """
    Discretized Mean Shift Clustering implementation based on the LaserNet paper
    titled "LaserNet: An Efficient Probabilistic 3D Object Detector for Autonomous Driving"
    https://arxiv.org/abs/1903.08701
    Section 3.4: Mean Shift Clustering
    """

    def __init__(
        self, nGridCells: List[int], nmsRadius: float, nIters: int = 3, minClusterPoints: int = 5
    ):
        """
        Parameters
        ----------
        nGridCells: list, len=2
            [N:int, M:int], number of rows and columns in the grid
        nmsRadius: float
            Radius to merge two clusters if the distance between them is smaller than this number
        nIters: int
            Number of iterations to run mean shift clustering
        minClusterPoints: int
            Minimum number of points per cluster to pass it
        """
        self.nGridCells = nGridCells
        self.nIters = nIters
        self.nmsRadius = nmsRadius
        self.minClusterPoints = minClusterPoints
        self.epsilon = 1e-5

        # To be initialized in the cluster method
        self.limits: Dict[str, List[float]] = {}
        self.gridCellHyp: float = 0

        # Parameter Validation
        try:
            assert len(nGridCells) == 2
            assert int(nGridCells[0]) == nGridCells[0] and nGridCells[0] > 0
            assert int(nGridCells[1]) == nGridCells[1] and nGridCells[1] > 0
            assert nmsRadius > 0
            assert int(nIters) == nIters and nIters > 0
            assert int(minClusterPoints) == minClusterPoints and minClusterPoints > 0
        except Exception as exc:
            errMsg = "Clusterer: parameter values invalid"
            raise TypeError(errMsg) from exc

    def cluster(self, points: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Run the clustering algorithm on the set of points

        Parameters
        ----------
        points: np.array, shape=(N,3)
            Points to cluster

        Returns
        -------
        means: np.array, shape=(C, 2)
            Centers of the clusters found, C is the number of clusters
        """
        points = points[:, :2]  # Only cluster using x and y locations
        self.limits = self.getLimits(points)

        totalGridCells = self.nGridCells[0] * self.nGridCells[1]
        gridCellSizeX = (self.limits["x"][1] - self.limits["x"][0]) / self.nGridCells[0]
        gridCellSizeY = (self.limits["y"][1] - self.limits["y"][0]) / self.nGridCells[1]
        self.gridCellHyp = gridCellSizeX**2 + gridCellSizeY**2

        # Construct initial gridMeans and gridCounts and populate them with points
        allCounts = np.zeros((totalGridCells))
        allMeans = np.zeros((totalGridCells, 2))
        locs = self.getLocs(points)  # Discretized locations for each point
        values, counts = np.unique(locs, return_counts=True)
        for val, count in zip(values, counts):
            allMeans[val] += np.sum(points[locs == val], axis=0)
            allCounts[val] += count
        allMeans[allCounts > 0] /= allCounts[allCounts > 0].reshape(-1, 1)

        # Run mean shifting for a number of iterations
        for _ in range(self.nIters):
            allMeans, allCounts = self.meanShiftIter(allMeans, allCounts)

        # Extract means according to counts in each cell
        if np.mean(allCounts) > self.minClusterPoints:
            passCountsFilter = allCounts > np.mean(allCounts)
        else:
            passCountsFilter = allCounts > self.minClusterPoints
        means: npt.NDArray[np.float64] = allMeans[passCountsFilter]

        means = self.radiusNMS(means)  # Can have a better formula for radius

        return means

    def meanShiftIter(
        self, allMeans: npt.NDArray[np.float64], allCounts: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Runs one iteration of mean shifting

        Parameters
        ----------
        allMeans: np.array of floats, shape=(N*M, 2), [N,M] is the grid shape
            Grid containing means of all the cells (but flattened the grid)
        allCounts: np.array of ints, shape=(N*M), [N,M] is the grid shape
            Grid containing the count of points in each cell (but flattened the grid)

        Returns
        -------
        newMeans: np.array, shape=(N*M, 2), [N,M] is the grid shape
            The new means after shifting the points and moving into new cells
        newCounts: np.array, shape=(N*M), [N,M] is the grid shape
            The new counts for each cell after shifting and moving into new cells
        """
        newMeans = self.getNewMean(allMeans, allCounts)

        # Locations if new means after shifting
        newMeansLocs = self.getLocs(newMeans[allCounts > 0])
        oldLocs = self.getLocs(allMeans[allCounts > 0])
        diffs = np.abs(newMeansLocs - oldLocs)
        shiftedPos = np.where(diffs > 0)[0]
        m_j_locs = newMeansLocs[shiftedPos]  # pylint: disable=invalid-name
        m_i_locs = oldLocs[shiftedPos]  # pylint: disable=invalid-name

        # Merge means that moved into new cells with the values in those cells
        shiftedMeans = np.copy(newMeans)
        shiftedMeans[m_i_locs] = 0

        newCounts = np.copy(allCounts)
        newCounts[m_i_locs] = 0
        for m_j, m_i in zip(m_j_locs, m_i_locs):  # pylint: disable=invalid-name
            shiftedMeans[m_j] = shiftedMeans[m_j] * newCounts[m_j] + newMeans[m_i] * allCounts[m_i]
            shiftedMeans[m_j] /= allCounts[m_i] + newCounts[m_j] + self.epsilon
            newCounts[m_j] = newCounts[m_j] + allCounts[m_i]
        return newMeans, newCounts

    def getNewMean(
        self, allMeans: npt.NDArray[np.float64], allCounts: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """
        Runs equations (3) and (4) from the paper mentioned in the class docstring

        Parameters
        ----------
        allMeans: np.array of floats, shape=(N*M, 2), [N,M] is the grid shape
            Grid containing means of all the cells (but flattened the grid)
        allCounts: np.array of ints, shape=(N*M), [N,M] is the grid shape
            Grid containing the count of points in each cell (but flattened the grid)

        Returns
        -------
        newMeans: np.array, shape=(N*M, 2), [N,M] is the grid shape
            The new means after shifting the points
        """
        gridMeans = allMeans.reshape(*self.nGridCells, 2)
        gridCounts = allCounts.reshape(*self.nGridCells)

        shiftedMeans = self.createShiftedVersion(gridMeans)
        shiftedCounts = self.createShiftedVersion(gridCounts)

        diffs = gridMeans.reshape(1, *self.nGridCells, 2) - shiftedMeans
        diffsNorm = np.linalg.norm(diffs, axis=3) ** 2
        K_i_j = np.exp(-1 * diffsNorm / self.gridCellHyp)  # pylint: disable=invalid-name

        weightedCounts = K_i_j * shiftedCounts
        newMeansDenom = np.sum(weightedCounts, axis=0)
        newMeansDenom = newMeansDenom.reshape(*self.nGridCells, 1)  # shape(N,M,2)

        newMeanNumer = weightedCounts.reshape(9, *self.nGridCells, 1) * shiftedMeans
        newMeanNumer = np.sum(newMeanNumer, axis=0)  # shape=(N,M,2)

        newMeans: npt.NDArray[np.float64] = newMeanNumer / (newMeansDenom + self.epsilon)
        newMeans = newMeans.reshape(-1, 2)
        return newMeans

    def createShiftedVersion(self, mat: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Given a matrix, create shifted versions of it in all directions.
        Ex: mat = [[1,2,3],
                    [4,5,6],
                    [7,8,9]]
        shifted_left = [[0,1,2],
                        [0,4,5],
                        [0,7,8]]
        shifted_up_right = [[5,6,0],
                            [7,8,0]
                            [0,0,0]]
        The function returns a list containing all 9 possible shifts (including original matrix)

        Parameters
        ----------
        mat: np.array, shape=(N,M,*)
            Matrix/Tensor to shift

        Returns
        -------
        shifted_mats: np.array, shape=(9,N,M,*)
            All shifted versions of the original matrix
        """
        nRows, nCols = mat.shape[:2]

        def shiftLeft(toShift: Any) -> Any:
            return np.hstack((np.zeros((nRows, 1, *toShift.shape[2:])), np.copy(toShift)))[:, :-1]

        def shiftRight(toShift: Any) -> Any:
            return np.hstack((np.copy(toShift), np.zeros((nRows, 1, *toShift.shape[2:]))))[:, 1:]

        def shiftUp(toShift: Any) -> Any:
            return np.vstack((np.zeros((1, nCols, *toShift.shape[2:])), np.copy(toShift)))[:-1, :]

        def shiftDown(toShift: Any) -> Any:
            return np.vstack((np.copy(toShift), np.zeros((1, nCols, *toShift.shape[2:]))))[1:, :]

        shiftedMats = [
            shiftLeft(shiftUp(mat)),
            shiftLeft(shiftDown(mat)),
            shiftRight(shiftUp(mat)),
            shiftRight(shiftDown(mat)),
            shiftLeft(mat),
            shiftRight(mat),
            shiftUp(mat),
            shiftDown(mat),
            np.copy(mat),
        ]
        return np.array(shiftedMats)

    def getLocs(self, allMeans: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Computes discretized grid locations for given points

        Parameters
        ----------
        allMeans: np.array (floats), shape=(N*M, 2), [N,M] are grid shape
            Each row contains the mean of points belonging to that cell
            If value = [0,0], then no points fall in that grid cell

        Returns
        -------
        locs: np.array (ints), shape=(N*M)
            The discretized locations in the grid.
            The value shows the index of where the means should fall.
            Indices represent a location in a 2D grid with size (N, M) that has been flattened
        """
        locs = np.copy(allMeans)
        locs[:, 0] -= self.limits["x"][0]
        locs[:, 1] -= self.limits["y"][0]
        locs[:, 0] /= self.limits["x"][1] - self.limits["x"][0]
        locs[:, 1] /= self.limits["y"][1] - self.limits["y"][0]
        locs[:, 0] *= self.nGridCells[0] - 1
        locs[:, 1] *= self.nGridCells[1] - 1
        locs = locs.astype(np.int32)
        locs = locs[:, 0] * self.nGridCells[1] + locs[:, 1]
        return locs

    def getLimits(self, points: npt.NDArray[np.float64]) -> Dict[str, List[float]]:
        """
        Gets x and y min and max values for a set of points

        Parameters
        ----------
        points: np.array, shape = (N,2)

        Returns
        -------
        limits: dict
        """
        xLimits = [points[:, 0].min(), points[:, 0].max()]
        yLimits = [points[:, 1].min(), points[:, 1].max()]
        limits = {"x": xLimits, "y": yLimits}
        return limits

    def radiusNMS(self, points: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Radius NMS, similar to euclidean clustering
        Points that are with (self.nmsRadius) of each other are merged into one cluster

        Parameters
        ----------
        points: np.array, shape=(N, 2)
            Each row is the position of a point

        Returns
        -------
        clusterMeans: np.array, shape=(C,2)
            Centers of the clusters found, C is the number of clusters
        """
        finalConesIdx = np.arange(0, points.shape[0])
        numPoints = points.shape[0]
        clusterMeans = []

        for _ in range(numPoints):
            if finalConesIdx.shape[0] == 0:  # Finished all points
                break
            origBoxId = finalConesIdx[0]
            origBox = points[origBoxId].reshape(1, 2)
            dist = np.linalg.norm(points[finalConesIdx] - origBox, axis=1)
            newCluster = finalConesIdx[np.where(dist <= self.nmsRadius)[0]]
            finalConesIdx = finalConesIdx[np.where(dist > self.nmsRadius)[0]]

            clusterMeans.append(np.mean(points[newCluster], axis=0))
        return np.array(clusterMeans)
