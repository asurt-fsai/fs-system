"""
Abstract class for ground removal methods
"""
from dataclasses import dataclass

import pcl

# To fix the issue with mypy not recognising the pcl classes
# These can be removed of course, they are just to make mypy happy
pcl.PointCloud = pcl.PointCloud


@dataclass
class GroundRemovalMethod:
    """
    Base class from ground removal methods
    """

    def removeGround(self, cloud: pcl.PointCloud) -> pcl.PointCloud:
        """
        Removes points falling onto the ground plane

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
