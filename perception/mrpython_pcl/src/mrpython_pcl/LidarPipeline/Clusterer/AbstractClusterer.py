"""
Abstract class Clusterer
"""
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass
class Clusterer:
    """
    Abstract Clusterer class for clustering methods
    """

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
        raise NotImplementedError
