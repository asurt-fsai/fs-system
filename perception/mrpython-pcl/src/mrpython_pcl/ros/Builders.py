"""
List of builder functions to instantiate different components of the pipeline with ros parameters
"""
from typing import Any

from asurt_msgs.msg import LandmarkArray  # pylint: disable=import-error
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

import rospy

from ..LidarPipeline.Filter.Filter import Filter
from ..LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from ..LidarPipeline.ConeClassifier.ConeClassifier import ConeClassifier
from ..LidarPipeline.Clusterer.MeanClusterer import MeanClusterer
from ..LidarPipeline.Clusterer.AbstractClusterer import Clusterer
from .LidarRosWrapper import LidarRosWrapper
from .MarkerViz import MarkerViz


class Builder:
    """
    Builder class to create the main pipeline object and publishers to use with it
    Uses ros parameters to create the different components of the pipeline
    """

    def __init__(self, isDefaultEnabled: bool = False):
        self.isDefaultEnabled = isDefaultEnabled

    def getParam(self, param: str, default: Any = None) -> Any:
        """
        Wrapper for the rospy.get_param function
        Prevents default values from being used if self.isDefaultEnabled is False

        Parameters
        ----------
        param : str
            Parameter name to fetch from the ros parameter server
        default : Any, optional
            If isDefaultEnabled is True and parameter name can't be found
            this value will be returned

        Returns
        -------
        Any
            Parameter value requested

        Raises
        ------
        Exception
            If parameter name can't be found and either:
                - isDefaultEnabled is False
                - isDefaultEnabled is True and default is None
        """
        if self.isDefaultEnabled and default is not None:
            return rospy.get_param(param, default)
        return rospy.get_param(param)

    def buildPipeline(self) -> LidarRosWrapper:
        """
        Creates the main pipeline object and publishers to use with it

        Returns
        -------
        lidarPipeline: LidarRosWrapper
            The main pipeline object created
        """
        filteredTopic = self.getParam("/perception/lidar/filtered_topic", "/tracked_cones")
        clusteredTopics = self.getParam("/perception/lidar/clustered_topic", "/tracked_cones")
        detectedConesTopic = self.getParam("/perception/lidar/detected_topic", "/tracked_cones")
        trackedConesTopic = self.getParam("/perception/lidar/tracked_topic", "/tracked_cones")

        publishers = {}
        publishers["filtered"] = rospy.Publisher(filteredTopic, PointCloud2, queue_size=10)
        publishers["clustered"] = rospy.Publisher(clusteredTopics, PointCloud2, queue_size=10)
        publishers["detected"] = rospy.Publisher(detectedConesTopic, LandmarkArray, queue_size=10)
        publishers["tracked"] = rospy.Publisher(trackedConesTopic, LandmarkArray, queue_size=10)
        publishers["detected_markers"] = rospy.Publisher(
            "/detected_cones_markers", MarkerArray, queue_size=10
        )

        markerViz = self.buildMarkerViz()
        filterer = self.buildFilter()
        clusterer = self.buildClusterer()
        coneClassifier = self.buildConeClassifier()
        tracker = None

        lidarPipeline = LidarRosWrapper(
            publishers, markerViz, filterer, clusterer, coneClassifier, tracker
        )

        velodyneTopic = self.getParam("/perception/lidar/velodyne_topic", "/velodyne_points")
        rospy.Subscriber(velodyneTopic, PointCloud2, callback=lidarPipeline.setPointcloud)

        return lidarPipeline

    def buildTracker(self) -> None:
        """
        max_age = self.getParam("max_age")
        min_hits = self.getParam("min_hits")
        min_iou_thresh = self.getParam("min_iou_thresh")

        class IoUTracker:
            def __init__(self, max_age, min_hits, min_iou_thresh):
                self.max_age = max_age
                self.min_hits = min_hits
                self.min_iou_thresh = min_iou_thresh

            def update(self, detections):
                return detections

        return IoUTracker(max_age, min_hits, min_iou_thresh)
        """
        return None

    def buildFilter(self) -> Filter:
        """
        Creates a filter along with its ground removal method using the ros parameters set

        Returns
        -------
        Filter
            The created Filter object
        """
        viewBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0], "z": [0.0, 0.0]}
        viewBounds["x"][0] = self.getParam("/perception/lidar/view_bounds/xmin", -10)
        viewBounds["x"][1] = self.getParam("/perception/lidar/view_bounds/xmax", 10)

        viewBounds["y"][0] = self.getParam("/perception/lidar/view_bounds/ymin", -6)
        viewBounds["y"][1] = self.getParam("/perception/lidar/view_bounds/ymax", 6)

        viewBounds["z"][0] = self.getParam("/perception/lidar/view_bounds/zmin", -2)
        viewBounds["z"][1] = self.getParam("/perception/lidar/view_bounds/xmax", 2)

        carBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0]}

        carBounds["x"][0] = self.getParam("/perception/lidar/car_bounds/xmin", -2)
        carBounds["x"][1] = self.getParam("/perception/lidar/car_bounds/xmax", 0)

        carBounds["y"][0] = self.getParam("/perception/lidar/car_bounds/ymin", -0.75)
        carBounds["y"][1] = self.getParam("/perception/lidar/car_bounds/ymax", 0.75)

        reconstructParam = self.getParam("/perception/lidar/cone_radius", 0.228)

        ransacThreshold = self.getParam("/perception/lidar/ransac_threshold", 0.1)
        lidarHeight = self.getParam("/perception/lidar/lidar_height", 0.1)
        groundRemover = SimpleGroundRemoval([0, 0, -1, -1 * lidarHeight], ransacThreshold, nIters=1)

        filterer = Filter(groundRemover, reconstructParam, viewBounds, carBounds)
        return filterer

    def buildConeClassifier(self) -> ConeClassifier:
        """
        Creates a cone classifier using the ros parameters set

        Returns
        -------
        ConeClassifier
            The created cone classifier object
        """
        coneRadius = self.getParam("/perception/lidar/cone_radius", 0.228)
        coneHeight = self.getParam("/perception/lidar/cone_height", 0.4)
        minPoints = self.getParam("/perception/lidar/cone_filter/min_points", 5)
        l2Th = self.getParam("/perception/lidar/cone_filter/l2_th", 0.03)
        linTh = self.getParam("/perception/lidar/cone_filter/lin_th", 1e-4)

        return ConeClassifier(coneRadius, coneHeight, minPoints, l2Th, linTh)

    def buildClusterer(self) -> Clusterer:
        """
        Creates a clusterer using the ros parameters set

        Returns
        -------
        Clusterer
            The created clusterer object

        Raises
        ------
        NotImplementedError
            When using cluster strategy "euclidean"
        ValueError
            When using cluster strategy other than the currently implemented:
            - mean_shift
        """
        clusterStrategy = self.getParam("/perception/lidar/cluster_strategy", "mean_shift")

        if clusterStrategy == "mean_shift":
            nGridCells = self.getParam("/perception/lidar/mean_shift/n_grid_cells", [40, 40])
            nmsRadius = self.getParam("/perception/lidar/mean_shift/nms_radius", 0.4)
            nIters = self.getParam("/perception/lidar/mean_shift/n_iters", 3)
            minClusterPoints = self.getParam("/perception/lidar/mean_shift/min_cluster_points", 5)
            return MeanClusterer(nGridCells, nmsRadius, nIters, minClusterPoints)

        if clusterStrategy == "euclidean":
            raise NotImplementedError(f"Invalid cluster strategy: {clusterStrategy}")

        raise ValueError(f"Invalid cluster strategy: {clusterStrategy}")

    def buildMarkerViz(self) -> MarkerViz:
        """
        Creates a marker viz object using the ros parameters set

        Returns
        -------
        MarkerViz
            The created marker viz object
        """
        coneRadius = self.getParam("/perception/lidar/cone_radius", 0.228)
        coneHeight = self.getParam("/perception/lidar/cone_height", 0.4)
        markerViz = MarkerViz(coneRadius, coneHeight)
        return markerViz
