"""
List of builder functions to instantiate different components of the pipeline with ros parameters
"""
from typing import Any, Dict

from asurt_msgs.msg import LandmarkArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

import rclpy
from tf_helper.MarkerViz import MarkerViz

from ..LidarPipeline.Filter.Filter import Filter
from ..LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from ..LidarPipeline.ConeClassifier.ConeClassifier import ConeClassifier
from ..LidarPipeline.Clusterer.MeanClusterer import MeanClusterer
from ..LidarPipeline.Clusterer.AbstractClusterer import Clusterer
from .LidarRosWrapper import LidarRosWrapper


class Builder:
    """
    Builder class to create the main pipeline object and publishers to use with it
    Uses ros parameters to create the different components of the pipeline
    """

    def __init__(self,node, isDefaultEnabled: bool = False):
        self.isDefaultEnabled = isDefaultEnabled
        self.node = node

    def getParam(self, param: str, default: Any = None) -> Any:
        """
        Wrapper for the rclpy.get_parameter function
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
            return self.node.get_parameter(param, default)
        return self.node.get_parameter(param)

    def buildPipeline(self) -> LidarRosWrapper:
        """
        Creates the main pipeline object and publishers to use with it

        Returns
        -------
        lidarPipeline: LidarRosWrapper
            The main pipeline object created
        """
        publishers = self.buildPublishers()

        markerViz = self.buildMarkerViz()
        filterer = self.buildFilter()
        clusterer = self.buildClusterer()
        coneClassifier = self.buildConeClassifier()
        tracker = None
        lidarHeight = self.getParam("/physical/lidar_height", 0.1)
        subsample = self.getParam("/perception/lidar/subsample", False)

        lidarPipeline = LidarRosWrapper(
            publishers,
            markerViz,
            filterer,
            clusterer,
            coneClassifier,
            tracker,
            lidarHeight,
            subsample,
        )

        velodyneTopic = self.getParam("/perception/lidar/velodyne_topic", "/velodyne_points")
        self.subscription = self.node.create_subscription(PointCloud2,velodyneTopic, callback=lidarPipeline.setPointcloud)

        return lidarPipeline

    def buildPublishers(self) -> Dict[str, rclpy.publisher]:
        """
        Creates the publishers to use with the main pipeline object

        Returns
        -------
        Dict[str, rclpy.publisher]
            A rospy publisher for each topic:
                filtered, clustered, detected, tracked, detected_markers
        """
        filteredTopic = self.getParam(
            "/perception/lidar/filtered_topic", "/placeholder/lidar/filtered"
        )
        clusteredTopics = self.getParam(
            "/perception/lidar/clustered_topic", "/placeholder/lidar/clustered"
        )
        detectedConesTopic = self.getParam(
            "/perception/lidar/detected_topic", "/placeholder/lidar/detected"
        )
        trackedConesTopic = self.getParam(
            "/perception/lidar/tracked_topic", "/placeholder/lidar/tracked"
        )
        detectedMarkersTopic = self.getParam(
            "/perception/lidar/detected_markers_topic", "/placeholder/lidar/detected_markers"
        )

        self.publishers = {}
        self.publishers["filtered"] = self.node.create_publisher( PointCloud2,filteredTopic, queue_size=10)
        self.publishers["clustered"] = self.node.create_publisher(PointCloud2,clusteredTopics,  queue_size=10)
        self.publishers["detected"] = self.node.create_publisher( LandmarkArray,detectedConesTopic, queue_size=10)
        self.publishers["tracked"] = self.node.create_publisher( LandmarkArray, trackedConesTopic,queue_size=10)
        self.publishers["detected_markers"] = self.node.create_publisher(
            detectedMarkersTopic, MarkerArray, queue_size=10
        )

        return self.publishers

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
        ransacThreshold = self.getParam("/perception/lidar/ransac_threshold", 0.1)
        lidarHeight = self.getParam("/physical/lidar_height", 0.1)

        viewBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0], "z": [0.0, 0.0]}
        viewBounds["x"][0] = self.getParam("/perception/lidar/view_bounds/xmin", -10)
        viewBounds["x"][1] = self.getParam("/perception/lidar/view_bounds/xmax", 10)

        viewBounds["y"][0] = self.getParam("/perception/lidar/view_bounds/ymin", -6)
        viewBounds["y"][1] = self.getParam("/perception/lidar/view_bounds/ymax", 6)

        viewBounds["z"][0] = -1 * lidarHeight + ransacThreshold
        viewBounds["z"][1] = self.getParam("/perception/lidar/view_bounds/zmax", 2)

        carBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0]}

        carBounds["x"][0] = self.getParam("/perception/lidar/car_bounds/xmin", -2)
        carBounds["x"][1] = self.getParam("/perception/lidar/car_bounds/xmax", 0)

        carBounds["y"][0] = self.getParam("/perception/lidar/car_bounds/ymin", -0.75)
        carBounds["y"][1] = self.getParam("/perception/lidar/car_bounds/ymax", 0.75)

        reconstructParam = self.getParam("/perception/lidar/reconst_radius", 0.228)

        groundRemover = SimpleGroundRemoval([0, 0, -1, -1 * lidarHeight], ransacThreshold, nIters=1)

        return Filter(groundRemover, reconstructParam, viewBounds, carBounds)

    def buildConeClassifier(self) -> ConeClassifier:
        """
        Creates a cone classifier using the ros parameters set

        Returns
        -------
        ConeClassifier
            The created cone classifier object
        """
        coneRadius = self.getParam("/physical/cone_radius", 0.228)
        coneHeight = self.getParam("/physical/cone_height", 0.4)
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
            nGridCellsX = self.getParam("/perception/lidar/mean_shift/n_grid_cells_x", 40)
            nGridCellsY = self.getParam("/perception/lidar/mean_shift/n_grid_cells_y", 40)
            nGridCells = [nGridCellsX, nGridCellsY]
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
        coneRadius = self.getParam("/physical/cone_radius", 0.228)
        coneHeight = self.getParam("/physical/cone_height", 0.4)
        lidarHeight = self.getParam("/physical/lidar_height", 0.1)
        markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)
        return markerViz
