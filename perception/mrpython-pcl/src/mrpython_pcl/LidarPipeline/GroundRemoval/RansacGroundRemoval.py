"""
Ransac based ground removal method
Based on the Ransac implementation in python-pcl
"""
from dataclasses import dataclass

import pcl
from ..helpers import SingletonMeta
from .GroundRemovalMethod import GroundRemovalMethod

# To fix the issue with mypy not recognising the pcl classes
# These can be removed of course, they are just to make mypy happy
pcl.PointCloud = pcl.PointCloud
pcl.SampleConsensusModelPlane = pcl.SampleConsensusModelPlane
pcl.RandomSampleConsensus = pcl.RandomSampleConsensus


@dataclass
class RansacGroundRemoval(GroundRemovalMethod, metaclass=SingletonMeta):
    """
    Removes points falling onto the ground plane using RANSAC implemented in python-pcl
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
