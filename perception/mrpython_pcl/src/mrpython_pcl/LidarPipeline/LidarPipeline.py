"""
LidarPipeline class to integrate the following classes:
    - Filter
    - Clusterer
    - ConeClassifier
    - Sort3D
"""
from typing import Optional, Dict, Any
from threading import Lock

from pcl import PointCloud
import numpy as np
import numpy.typing as npt

from .Filter.Filter import Filter
from .ConeClassifier.ConeClassifier import ConeClassifier
from .Clusterer.AbstractClusterer import Clusterer
from .helpers import SingletonMeta, mutexLock


class LidarPipeline(metaclass=SingletonMeta):
    """
    Main Lidar Pipeline class
    """

    mutex = Lock()

    def __init__(
        self,
        filterer: Filter,
        clusterer: Clusterer,
        coneClassifier: ConeClassifier,
        tracker: Any = None,
        lidarHeight: float = 0,
        subsample: bool = False,
    ):
        self.filterer = filterer
        self.clusterer = clusterer
        self.coneClassifier = coneClassifier
        self.tracker = tracker
        self.lidarHeight = lidarHeight
        self.subsample = subsample

        self.lidarPc = None

        # Parameter validation
        try:
            assert isinstance(filterer, Filter)
            assert isinstance(clusterer, Clusterer)
            assert isinstance(coneClassifier, ConeClassifier)
            assert type(lidarHeight) in [int, float]
            assert lidarHeight >= 0
        except Exception as exp:
            errMsg = "LidarPipeline: ensure all parameters are correct \n \
                        filterer, clusterer, coneClassifier should have classes \
                            Filter, Clusterer, ConeClassifier respectively \n\
                        lidarHeight: float > 0"
            raise TypeError(errMsg) from exp

    @mutexLock(mutex)
    def setPointcloud(self, pointcloud: PointCloud) -> None:
        """
        Set the point cloud to be used in the pipeline

        Parameters
        ----------
        pointcloud : PointCloud
            Raw point cloud recieved from the lidar
        """
        self.lidarPc = pointcloud

    @mutexLock(mutex)
    def getPointcloud(self) -> Optional[PointCloud]:
        """
        Fetches the latest point cloud and sets it to None

        Returns
        -------
        PointCloud or None
            Raw point cloud recieved from the lidar
            Returns None if no point cloud has been added
        """
        toReturn = self.lidarPc
        self.lidarPc = None
        return toReturn

    def run(
        self,
    ) -> Optional[Dict[str, npt.NDArray[np.float64]]]:
        """
        Run the lidar pipeline using the latest added pointcloud

        Returns
        -------
        dict
            Dictionary containing the following keys:
                - "filtered": Filtered pointcloud
                - "clustered": Point cloud of points containing only the reconstructed clusters
                - "clusterCenters": List of cluster centers (2D)
                - "detected": List of detected cone centers
                - "tracked": List of tracked cone centers
        """
        cloud = self.getPointcloud()
        if cloud is None:
            return None

        # Preprocessing: filtering the point cloud
        cloud = self.filterer.removeIntensity(cloud)
        cloud = self.filterer.filterViewableArea(cloud)
        cloud = self.filterer.removeCar(cloud)
        cloud = self.filterer.removeGround(cloud)
        if self.subsample:
            cloud = self.filterer.subsample(cloud)
        cloudFil = cloud.to_array()

        clusterCenters = self.clusterer.cluster(cloudFil)

        clusters = np.empty((0, 3))
        cones = np.empty((0, 3))

        # Extract true cones and their centers from the clusters
        for center in clusterCenters:
            points = self.filterer.reconstruct(cloud, center[0], center[1])
            points = points.to_array()

            status, coneCentroids = self.coneClassifier.isCone(points)
            clusters = np.vstack((clusters, points))
            if status[0] and coneCentroids is not None:
                if len(cones) == 0:
                    cones = coneCentroids
                else:
                    cones = np.vstack((cones, coneCentroids))

        # Shift cone centers down to actual center
        cones[:, 2] = -self.lidarHeight

        output = {}
        output["filtered"] = cloudFil
        output["clustered"] = clusters
        output["clusterCenters"] = clusterCenters
        output["detected"] = cones

        # Update trackers, and add them to message
        if self.tracker is not None:
            if len(cones) > 0:
                trackedCones = self.tracker.update(np.array(cones))
            else:
                trackedCones = self.tracker.update()

            output["tracked"] = trackedCones

        return output
