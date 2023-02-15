"""
Abstract class for ground removal methods
"""
from dataclasses import dataclass

from pcl import PointCloud


@dataclass
class GroundRemovalMethod:
    """
    Base class from ground removal methods
    """

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
        raise NotImplementedError
