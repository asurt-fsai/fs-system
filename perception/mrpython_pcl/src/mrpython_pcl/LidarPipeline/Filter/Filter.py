"""
A set of filters used including:
    - Pass view filter
    - Removing points falling onto the car
    - RANSAC to remove the ground
    - Subsampling filter
    - Cone reconstruction filter
"""
from typing import Dict, List

from pcl import PointCloud, PointCloud_PointXYZI
import numpy as np
from ..helpers import SingletonMeta
from ..GroundRemoval.GroundRemovalMethod import GroundRemovalMethod


class Filter(metaclass=SingletonMeta):
    """
    Filter class containing many preprocessing methods for the point cloud
    """

    def __init__(
        self,
        groundRemovalMethod: GroundRemovalMethod,
        resconstructParam: float,
        viewableBounds: Dict[str, List[float]],
        carDimensions: Dict[str, List[float]],
    ):
        """
        Parameters
        ----------
        groundRemovalMethod: GroundRemovalMethod
            Class to use for removing points falling on the ground
        reconstruct_radius: float
            Radius to reconstruct around the given center
        viewableBounds: dict
            Contains the boundaries as a 2D box to use as the viewable area.
            It should have the followign structure:
            {
                "x": [xMin, xMax],
                "y": [yMin, yMax],
                "z": [z_min, z_max]
            }
        carDimensions: dict
            Contains the boundaries of a 2D box to use to remove points within
            the vehicle's footprint. It should have a structure similar to viewableBounds.
            Note: The z-axis is not used in this filter
        """
        self.groundRemovalMethod = groundRemovalMethod
        self.resconstructParam = resconstructParam
        self.viewableBounds = viewableBounds
        self.carDimensions = carDimensions

        # Parameter Validation
        try:
            assert resconstructParam > 0 and type(resconstructParam) in [int, float]
            assert isinstance(groundRemovalMethod, GroundRemovalMethod)

            def checkDict(toCheckDict: Dict[str, List[float]], checkZ: bool = False) -> None:
                """
                Checks if a dict satistfies the requirements for
                the viewableBounds and carDimensions

                Parameters
                ----------
                toCheckDict : Dict[str, List[float]]
                    Dictionary to check
                checkZ : bool, optional
                    If true, checks also the z limits in dictionary, by default False
                """
                assert isinstance(toCheckDict, dict)
                assert "x" in toCheckDict.keys()
                assert "y" in toCheckDict.keys()
                assert len(toCheckDict["x"]) == 2
                assert len(toCheckDict["y"]) == 2
                assert type(toCheckDict["x"][0]) in [int, float]
                assert type(toCheckDict["x"][1]) in [int, float]
                assert type(toCheckDict["y"][0]) in [int, float]
                assert type(toCheckDict["y"][1]) in [int, float]
                assert toCheckDict["x"][0] < toCheckDict["x"][1]
                assert toCheckDict["y"][0] < toCheckDict["y"][1]
                if checkZ:
                    assert "z" in toCheckDict.keys()
                    assert len(toCheckDict["z"]) == 2
                    assert type(toCheckDict["z"][0]) in [int, float]
                    assert type(toCheckDict["z"][1]) in [int, float]
                    assert toCheckDict["z"][0] < toCheckDict["z"][1]

            checkDict(viewableBounds, checkZ=True)
            checkDict(carDimensions)
        except Exception as exp:
            errMsg = "Filter: ensure all parameters are within correct ranges \n \
                        reconstruct_radius: float > 0 \
                        viewableBounds and carDimensions: dict, see docstring"
            raise TypeError(errMsg) from exp

    def filterViewableArea(self, cloud: PointCloud) -> PointCloud:
        """
        Filters to point cloud to include the viewable area (box around the origin)

        Parameters
        ----------
        cloud: PointCloud
            Point cloud to filter

        Returns
        -------
        cloud: PointCloud
            Filtered point cloud
        """
        cloud = self.passFilter(cloud, "y", *self.viewableBounds["y"])
        cloud = self.passFilter(cloud, "x", *self.viewableBounds["x"])
        cloud = self.passFilter(cloud, "z", *self.viewableBounds["z"])
        return cloud

    def removeCar(self, cloud: PointCloud) -> PointCloud:
        """
        Removes points that possibly fall on the car (ego)

        Parameters
        ----------
        cloud: PointCloud
            Point cloud to clip

        Returns
        -------
        cloud: PointCloud
            Point cloud with the 2D box clipped from it
        """
        xMin = self.carDimensions["x"][0]
        xMax = self.carDimensions["x"][1]
        yMin = self.carDimensions["y"][0]
        yMax = self.carDimensions["y"][1]

        cloudsArr = cloud.to_array()
        idxX = np.logical_or(cloudsArr[:, 0] > xMax, cloudsArr[:, 0] < xMin)
        idxY = np.logical_or(cloudsArr[:, 1] > yMax, cloudsArr[:, 1] < yMin)
        idx = np.logical_or(idxX, idxY)
        idx = np.where(idx)[0]
        cloud = PointCloud()
        cloud.from_array(cloudsArr[idx])

        return cloud

    def removeGround(self, cloud: PointCloud) -> PointCloud:
        """
        Removes points falling onto the ground plance

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
        filteredCloud = self.groundRemovalMethod.removeGround(cloud)
        return filteredCloud

    def reconstruct(self, cloud: PointCloud, centerX: float, centerY: float) -> PointCloud:
        """
        Returns points from the given problem within a box (on x and y axes)
        around the given center point

        Parameters
        ----------
        cloud: PointCloud
            Point cloud to fetch points from
        centerX: float
            X-axis center
        centerY: float
            Y-axis center

        Returns
        -------
        cloud: PointCloud
            Points with a box around the given center (centerX, centerY)
        """
        xmin = centerX - self.resconstructParam
        xmax = centerX + self.resconstructParam
        ymin = centerY - self.resconstructParam
        ymax = centerY + self.resconstructParam

        # y filter (lateral)
        cloud = self.passFilter(cloud, "y", ymin, ymax)

        # x filter (longitudinal)
        cloud = self.passFilter(cloud, "x", xmin, xmax)

        return cloud

    @staticmethod
    def passFilter(cloud: PointCloud, field: str, minLimit: float, maxLimit: float) -> PointCloud:
        """
        Applies a pass filter on a cloud on a particular axis. It passes values only
        between minLimit and maxLimit on the given axis (field)

        Parameters
        ----------
        cloud: PointCloud
            Point cloud to filter
        field: str
            field for which to apply filter e.g. "x", "y", "z"
        minLimit: float
            The minimum limit of the filter
        maxLimit: float
            The maximum limit of the filter

        Returns
        -------
        cloud: PointCloud
            Filtered point cloud
        """
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name(field)
        passthrough.set_filter_limits(minLimit, maxLimit)
        cloud = passthrough.filter()
        return cloud

    @staticmethod
    def subsample(cloud: PointCloud, ratio: float = 0.8, temperature: float = 500) -> PointCloud:
        """
        Subsample a ratio of points from the pointcloud weighted, where near
        points are more likely to be sampled
        TODO: can speed up by using cloud.extract instead of cloud.from_array?

        Parameters
        ----------
        cloud: PointCloud
            Point cloud to subsample from
        ratio: float, optional, default 0.8
            Fraction of points to sample from the point cloud
        temprature: float, optional, default 500
            Temperature for the "softmax-like" function used for sampling

        Returns
        -------
        cloud: PointCloud
            Subsampled point cloud
        """
        assert 0 < ratio <= 1

        cloudArr = cloud.to_array()
        if cloudArr.shape[0] == 0:  # All points were filtered out
            return cloud

        # Compute sampling weights (probs)
        dists = np.linalg.norm(cloudArr, axis=1)
        probs = dists * temperature
        probs /= np.sum(probs)

        # Subsample
        nNewSamples = int(cloudArr.shape[0] * ratio)
        sampledIndices = np.random.choice(cloudArr.shape[0], nNewSamples, replace=False, p=probs)
        cloudArr = cloudArr[sampledIndices]

        cloud = PointCloud()
        cloud.from_array(cloudArr)
        return cloud

    @staticmethod
    def removeIntensity(cloud: PointCloud_PointXYZI) -> PointCloud:
        """
        Removes the intensity information from a point cloud, leaving only x, y, z

        Parameters
        ----------
        cloud: PointCloud_PointXYZI
            Point cloud with intensity (i.e. each point has x,y,z,i)

        Returns
        -------
        cloud: PointCloud
            Point cloud with intensity removed
        """
        cloudArr = cloud.to_array()
        cloud = PointCloud()
        cloud.from_array(cloudArr[:, :3])
        return cloud
