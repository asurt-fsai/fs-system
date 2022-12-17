"""
List of builder functions to instantiate different components of the pipeline with ros parameters
"""
from asurt_msgs.msg import LandmarkArray  # pylint: disable=import-error
from sensor_msgs.msg import PointCloud2

import rospy

from ...modules.Filter.Filter import Filter
from ...modules.Filter.GroundRemoval import RansacGroundRemoval
from ...modules.ConeClassifier.ConeClassifier import ConeClassifier
from ...modules.Clusterer.MeanClusterer import MeanClusterer
from ...modules.Clusterer.AbstractClusterer import Clusterer
from .LidarRosWrapper import LidarRosWrapper


def buildPipeline() -> LidarRosWrapper:
    """
    Creates the main pipeline object and publishers to use with it

    Returns
    -------
    lidarPipeline: LidarRosWrapper
        The main pipeline object created
    """
    filteredTopic = rospy.get_param("perception/lidar/filteredTopic", "/tracked_cones")
    clusteredTopics = rospy.get_param("perception/lidar/clusteredTopics", "/tracked_cones")
    detectedConesTopic = rospy.get_param("perception/lidar/detectedConesTopic", "/tracked_cones")
    trackedConesTopic = rospy.get_param("perception/lidar/trackedConesTopic", "/tracked_cones")

    publishers = {}
    publishers["filtered"] = rospy.Publisher(filteredTopic, PointCloud2, queue_size=10)
    publishers["clustered"] = rospy.Publisher(clusteredTopics, PointCloud2, queue_size=10)
    publishers["detected"] = rospy.Publisher(detectedConesTopic, LandmarkArray, queue_size=10)
    publishers["tracked"] = rospy.Publisher(trackedConesTopic, LandmarkArray, queue_size=10)

    filterer = buildFilter()
    clusterer = buildClusterer()
    coneClassifier = buildConeClassifier()
    tracker = None

    lidarPipeline = LidarRosWrapper(publishers, filterer, clusterer, coneClassifier, tracker)

    velodyneTopic = rospy.get_param("perception/lidar/velodyneTopic", "/velodyne_points")
    rospy.Subscriber(velodyneTopic, PointCloud2, callback=lidarPipeline.setPointcloud)

    return lidarPipeline


def buildTracker() -> None:
    """
    max_age = rospy.get_param("max_age")
    min_hits = rospy.get_param("min_hits")
    min_iou_thresh = rospy.get_param("min_iou_thresh")

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


def buildFilter() -> Filter:
    """
    Creates a filter along with its ground removal method using the ros parameters set

    Returns
    -------
    Filter
        The created Filter object
    """
    viewBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0], "z": [0.0, 0.0]}
    viewBounds["x"][0] = rospy.get_param("/perception/lidar/view_bounds/xmin", -10)
    viewBounds["x"][1] = rospy.get_param("/perception/lidar/view_bounds/xmax", 10)

    viewBounds["y"][0] = rospy.get_param("/perception/lidar/view_bounds/ymin", -6)
    viewBounds["y"][1] = rospy.get_param("/perception/lidar/view_bounds/ymax", 6)

    viewBounds["z"][0] = rospy.get_param("/perception/lidar/view_bounds/zmin", -2)
    viewBounds["z"][1] = rospy.get_param("/perception/lidar/view_bounds/xmax", 2)

    carBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0]}

    carBounds["x"][0] = rospy.get_param("/perception/lidar/car_bounds/xmin", -2)
    carBounds["x"][1] = rospy.get_param("/perception/lidar/car_bounds/xmax", 0)

    carBounds["y"][0] = rospy.get_param("/perception/lidar/car_bounds/ymin", -0.75)
    carBounds["y"][1] = rospy.get_param("/perception/lidar/car_bounds/ymax", 0.75)

    reconstructParam = rospy.get_param("/perception/lidar/cone_radius", 0.228)

    ransacThreshold = rospy.get_param("/perception/lidar/ransac_threshold", 0.1)
    groundRemover = RansacGroundRemoval(ransacThreshold)

    filterer = Filter(groundRemover, reconstructParam, viewBounds, carBounds)
    return filterer


def buildConeClassifier() -> ConeClassifier:
    """
    Creates a cone classifier using the ros parameters set

    Returns
    -------
    ConeClassifier
        The created cone classifier object
    """
    coneRadius = rospy.get_param("/perception/lidar/cone_radius", 0.228)
    coneHeight = rospy.get_param("/perception/lidar/cone_height", 0.228)
    minPoints = rospy.get_param("/perception/lidar/cone_filter/min_points", 5)
    l2Th = rospy.get_param("/perception/lidar/cone_filter/l2_th", 0.03)
    linTh = rospy.get_param("/perception/lidar/cone_filter/lin_th", 1e-4)

    return ConeClassifier(coneRadius, coneHeight, minPoints, l2Th, linTh)


def buildClusterer() -> Clusterer:
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
    clusterStrategy = rospy.get_param("/perception/lidar/cluster_strategy", "mean_shift")

    if clusterStrategy == "mean_shift":
        nGridCells = rospy.get_param("/perception/lidar/mean_shift/n_grid_cells", [40, 40])
        nmsRadius = rospy.get_param("/perception/lidar/mean_shift/nms_radius", 0.4)
        nIters = rospy.get_param("/perception/lidar/mean_shift/n_iters", 3)
        minClusterPoints = rospy.get_param("/perception/lidar/mean_shift/min_cluster_points", 5)
        return MeanClusterer(nGridCells, nmsRadius, nIters, minClusterPoints)

    if clusterStrategy == "euclidean":
        raise NotImplementedError(f"Invalid cluster strategy: {clusterStrategy}")
        # cluster_tolerance = rospy.get_param("/perception/lidar/euclidean/cluster_tolerance", 0.2)
        # cluster_min_points = rospy.get_param("/perception/lidar/euclidean/cluster_min_points", 5)
        # cluster_max_points = rospy.get_param("/perception/lidar/euclidean/cluster_max_points", 40)
        # return EuclideanClusterer(cluster_tolerance, cluster_min_points, cluster_max_points)

    raise ValueError(f"Invalid cluster strategy: {clusterStrategy}")
