"""
Description: This File calculates the Adjacency Matrix
"""

from typing import Tuple, cast

import numpy as np

from planning_centerline_calculation.types import FloatArray, IntArray
from planning_centerline_calculation.utils.cone_types import ConeTypes, invertConeType
from planning_centerline_calculation.utils.math_utils import calcPairwiseDistances



class AdjacencyMatrix:
    """
    A class to create and calculate an adjacency matrix
    """
    
    LAST_MATRIX_CALC_HASH = None
    LAST_MATRIX_CALC_DISTANCE_MATRIX = None

    LAST_IDXS_CALCULATED = None
    LAST_IDXS_HASH = None
    def __init__(
            self,
            maxDist: float,
    ) -> None:
        """
        Constructor for AdjacencyMatrix class
        """
        self.maxDist = maxDist


    def createAdjacencyMatrix(
            self,
            cones: FloatArray,
            nNeighbors: int,
            startIdx: int,
            coneType: ConeTypes,
    ) -> Tuple[IntArray, IntArray]:
        """
        Creates the adjacency matrix that defines the possible points each point can be connected with
        Args:
            cones: The trace containing all the points
            n_neighbors: The maximum number of neighbors each node can have
            start_idx: The index from which the trace starts
            max_dist: The maximum distance two points can have in order for them to
            be considered possible neighbors
        Returns:
            Tuple[np.array, np.array]: Three values are returned. First a square boolean
            matrix indicating at each position if two nodes are connected. The second 1d
            matrix contains the reachable nodes from `start_idx`.
        """

        nPoints = cones.shape[0]

        conesXY = cones[:, :2]
        ConesColor = cones[:, 2]

        pairwiseDistance: FloatArray = self.calcDistanceMatrix(conesXY)

        maskIsOtherConeType = ConesColor == invertConeType(coneType)
        pairwiseDistance[maskIsOtherConeType, :] = np.inf
        pairwiseDistance[:, maskIsOtherConeType] = np.inf

        kClosestEach = self.findKClosestInPointCloud(pairwiseDistance, nNeighbors)

        sources = np.repeat(np.arange(nPoints), nNeighbors)
        targets = kClosestEach.flatten()

        adjacecnyMatrix: IntArray = np.zeros((nPoints, nPoints), dtype=np.uint8)
        adjacecnyMatrix[sources, targets] = 1 # for each node set its closest nNeighbors to 1
        adjacecnyMatrix[pairwiseDistance > (self.maxDist * self.maxDist)] = 0 # but if distance is too high set to 0 again

        #remove all egdes that dont't have a revere i.e. convert to undirected graph
        adjacecnyMatrix = np.logical_and(adjacecnyMatrix, adjacecnyMatrix.T)

        reachableNodes = self.breadthFirstOrder(adjacecnyMatrix, startIdx)

        # # completely disconnect nodes that are not reachable from the start node
        # # assume that all nodes will be disconnected
        # nodesToDisconnect = np.ones(nPoints, dtype=bool)
        # # but for the reachable nodes don't do anything
        # nodesToDisconnect[reachableNodes] = False

        return adjacecnyMatrix, reachableNodes

    def calcDistanceMatrix(self, conesXY: FloatArray) -> FloatArray:
        """
        Calculates the distance matrix between all points in the trace
        Args:
            conesXY: The trace containing
        """
        inputHash = hash(conesXY.tobytes())
        if inputHash != self.LAST_MATRIX_CALC_HASH:
            self.LAST_MATRIX_CALC_HASH = inputHash
            self.LAST_MATRIX_CALC_DISTANCE_MATRIX = calcPairwiseDistances(
                conesXY, dist_to_self=np.inf
            )
        
        return self.LAST_MATRIX_CALC_DISTANCE_MATRIX.copy()

    def findKClosestInPointCloud(self, pairwiseDistance: FloatArray, k:int) -> IntArray:
            """
            Finds the indices of the k closest points for each point in a point cloud from its
            pairwise distances.

            Args:
                pairwise_distances: A square matrix containing the distance from each
                point to every other point
                k: The number closest points (indices) to return of each point
            Returns:
                np.array: An (n,k) array containing the indices of the `k` closest points.
            """

            inputHash = hash((pairwiseDistance.tobytes(), k))
            #Check if input has changed
            if inputHash != self.LAST_IDXS_HASH:
                 self.LAST_IDXS_HASH = inputHash
                 self.LAST_INDXS_CALCULATED = np.argsort(pairwiseDistance, axis=1)[:, :k]

            return self.LAST_INDXS_CALCULATED.copy()
    
    def breadthFirstOrder(self, adjacencyMatrix: IntArray, startIdx: int) -> IntArray:
        """
        Returns the nodes reachable from `start_idx` in BFS order
        Args:
            adjacency_matrix: The adjacency matrix describing the graph
            start_idx: The index of the starting node
        Returns:
            np.array: An array containing the nodes reachable from the starting node in BFS order
        """
        visited = np.zeros(adjacencyMatrix.shape[0], dtype=np.uint8)
        queue = np.full(adjacencyMatrix.shape[0], fill_value=-1)

        queue[0] = startIdx
        visited[startIdx] = 1

        queue_pointer = 0
        queue_end_pointer = 0

        while queue_pointer <= queue_end_pointer:
            node = queue[queue_pointer]

            next_nodes = np.argwhere(adjacencyMatrix[node])[:, 0]
            for i in next_nodes:
                if not visited[i]:
                    queue_end_pointer += 1
                    queue[queue_end_pointer] = i
                    visited[i] = 1
            queue_pointer += 1
        return cast(IntArray, queue[:queue_pointer])

    