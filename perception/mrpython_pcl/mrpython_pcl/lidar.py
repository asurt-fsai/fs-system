#!/usr/bin/python3
"""
Main ros node for the lidar pipeline used to detect cones
"""
import rclpy
from rclpy.node import Node
from typing import Any, Dict

from asurt_msgs.msg import LandmarkArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

# from tf_helper.MarkerViz import MarkerViz
from smoreo.MarkerViz import MarkerViz

from .LidarPipeline.Filter.Filter import Filter
from .LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from .LidarPipeline.ConeClassifier.ConeClassifier import ConeClassifier
from .LidarPipeline.Clusterer.MeanClusterer import MeanClusterer
from .LidarPipeline.Clusterer.AbstractClusterer import Clusterer
from .LidarRosWrapper import LidarRosWrapper
# from mrpython_pcl.ros.Builders import Builder
# from tf_helper.StatusPublisher import StatusPublisher

class LidarSystem(Node):
    def __init__(self, isDefaultEnabled: bool = False):
        super().__init__("lidar")
       
        self.isDefaultEnabled  = isDefaultEnabled

        self.publishers_filtered: rclpy.publisher.Publisher
        self.publishers_clustered: rclpy.publisher.Publisher
        self.publishers_detected: rclpy.publisher.Publisher
        self.publishers_tracked: rclpy.publisher.Publisher
        self.publishers_detected_markers: rclpy.publisher.Publisher

        self.declare_parameters(
            namespace='',
            parameters=[  

                ('/physical/lidar_height', rclpy.Parameter.Type.DOUBLE),
                ('/physical/cone_height', rclpy.Parameter.Type.DOUBLE),
                ('/physical/cone_radius', rclpy.Parameter.Type.DOUBLE),
                                
                ('/perception/lidar/subsample', rclpy.Parameter.Type.BOOL),
                ('/perception/lidar/velodyne_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/filtered_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/clustered_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/detected_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/tracked_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/detected_markers_topic', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/ransac_threshold', rclpy.Parameter.Type.DOUBLE),

                ('/perception/lidar/view_bounds/xmin', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/view_bounds/xmax', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/view_bounds/ymin', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/view_bounds/ymax', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/view_bounds/zmax', rclpy.Parameter.Type.INTEGER),

                ('/perception/lidar/car_bounds/xmin', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/car_bounds/xmax', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/car_bounds/ymin', rclpy.Parameter.Type.DOUBLE),
                ('/perception/lidar/car_bounds/ymax', rclpy.Parameter.Type.DOUBLE),
                ('/perception/lidar/reconst_radius', rclpy.Parameter.Type.DOUBLE),
                ('/perception/lidar/cluster_strategy', rclpy.Parameter.Type.STRING),
                ('/perception/lidar/mean_shift/n_grid_cells_x', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/mean_shift/n_grid_cells_y', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/mean_shift/nms_radius', rclpy.Parameter.Type.DOUBLE),
                ('/perception/lidar/mean_shift/n_iters', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/mean_shift/min_cluster_points', rclpy.Parameter.Type.INTEGER),
                
                ('/perception/lidar/cone_filter/min_points', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/cone_filter/l2_th', rclpy.Parameter.Type.DOUBLE),
                ('/perception/lidar/cone_filter/lin_th', rclpy.Parameter.Type.DOUBLE),


            ]
            )

        
        self.create_timer(0.1, self.timer_callback)
        # self.status = StatusPublisher("/status/lidar")
        # self.status.starting()
        self.lidar = self.buildPipeline()
        # self.status.ready()

    

    # def getParam(self, param: str, default: Any = None) -> Any:
    #     """
    #     Wrapper for the rclpy.get_parameter function
    #     Prevents default values from being used if self.isDefaultEnabled is False

    #     Parameters
    #     ----------
    #     param : str
    #         Parameter name to fetch from the ros parameter server
    #     default : Any, optional
    #         If isDefaultEnabled is True and parameter name can't be found
    #         this value will be returned

    #     Returns
    #     -------
    #     Any
    #         Parameter value requested

    #     Raises
    #     ------
    #     Exception
    #         If parameter name can't be found and either:
    #             - isDefaultEnabled is False
    #             - isDefaultEnabled is True and default is None
    #     """
    #     # if self.isDefaultEnabled and default is not None:
    #         # return self.get_parameter(param, default).get_parameter_value().string_value
    #     return self.get_parameter(param).get_parameter_value().string_value

    def buildPipeline(self) -> LidarRosWrapper:
        """
        Creates the main pipeline object and publishers to use with it

        Returns
        -------
        lidarPipeline: LidarRosWrapper
            The main pipeline object created
        """
        # publishers = self.buildPublishers()

        markerViz = self.buildMarkerViz()
        filterer = self.buildFilter()
        clusterer = self.buildClusterer()
        coneClassifier = self.buildConeClassifier()
        tracker = None
        lidarHeight = self.get_parameter("/physical/lidar_height").get_parameter_value().double_value
        subsample = self.get_parameter("/perception/lidar/subsample").get_parameter_value().bool_value  
        publishers_filtered, publishers_clustered, publishers_detected, publishers_tracked, publishers_detected_markers= self.buildPublishers()
        lidarPipeline = LidarRosWrapper(
            # publishers,
            markerViz,
            publishers_filtered,
            publishers_clustered,
            publishers_detected,
            publishers_tracked,
            publishers_detected_markers,
            filterer,
            clusterer,
            coneClassifier,
            tracker,
            lidarHeight,
            subsample,
        )

        velodyneTopic = self.get_parameter("/perception/lidar/velodyne_topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(PointCloud2,velodyneTopic, callback=lidarPipeline.setPointcloud, qos_profile=10)

        return lidarPipeline

    def buildPublishers(self):
        """
        Creates the publishers to use with the main pipeline object

        Returns
        -------
        Dict[str, rclpy.publisher]
            A rospy publisher for each topic:
                filtered, clustered, detected, tracked, detected_markers
        """
        filteredTopic = self.get_parameter("/perception/lidar/filtered_topic").get_parameter_value().string_value
        clusteredTopics = self.get_parameter("/perception/lidar/clustered_topic").get_parameter_value().string_value
        detectedConesTopic = self.get_parameter("/perception/lidar/detected_topic").get_parameter_value().string_value
        trackedConesTopic = self.get_parameter("/perception/lidar/tracked_topic").get_parameter_value().string_value
        detectedMarkersTopic = self.get_parameter("/perception/lidar/detected_markers_topic").get_parameter_value().string_value

     
        self.publishers_filtered = self.create_publisher( PointCloud2,filteredTopic, 10)
        self.publishers_clustered = self.create_publisher(PointCloud2,clusteredTopics,  10)
        self.publishers_detected = self.create_publisher( LandmarkArray,detectedConesTopic, 10)
        self.publishers_tracked = self.create_publisher( LandmarkArray, trackedConesTopic,10)
        self.publishers_detected_markers = self.create_publisher(MarkerArray,detectedMarkersTopic,  10)

        return self.publishers_filtered, self.publishers_clustered, self.publishers_detected, self.publishers_tracked, self.publishers_detected_markers

    def buildTracker(self) -> None:
        """
        max_age = self.get_parameter("max_age")
        min_hits = self.get_parameter("min_hits")
        min_iou_thresh = self.get_parameter("min_iou_thresh")

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
        ransacThreshold = self.get_parameter("/perception/lidar/ransac_threshold").get_parameter_value().double_value
        lidarHeight = self.get_parameter("/physical/lidar_height").get_parameter_value().double_value

        viewBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0], "z": [0.0, 0.0]}
        viewBounds["x"][0] = self.get_parameter("/perception/lidar/view_bounds/xmin").get_parameter_value().integer_value
        viewBounds["x"][1] = self.get_parameter("/perception/lidar/view_bounds/xmax").get_parameter_value().integer_value

        viewBounds["y"][0] = self.get_parameter("/perception/lidar/view_bounds/ymin").get_parameter_value().integer_value
        viewBounds["y"][1] = self.get_parameter("/perception/lidar/view_bounds/ymax").get_parameter_value().integer_value

        viewBounds["z"][0] = -1 * lidarHeight + ransacThreshold
        viewBounds["z"][1] = self.get_parameter("/perception/lidar/view_bounds/zmax").get_parameter_value().integer_value

        carBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0]}

        carBounds["x"][0] = self.get_parameter("/perception/lidar/car_bounds/xmin").get_parameter_value().integer_value
        carBounds["x"][1] = self.get_parameter("/perception/lidar/car_bounds/xmax").get_parameter_value().integer_value  

        carBounds["y"][0] = self.get_parameter("/perception/lidar/car_bounds/ymin").get_parameter_value().double_value
        carBounds["y"][1] = self.get_parameter("/perception/lidar/car_bounds/ymax").get_parameter_value().double_value

        reconstructParam = self.get_parameter("/perception/lidar/reconst_radius").get_parameter_value().double_value    

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
        coneRadius = self.get_parameter("/physical/cone_radius").get_parameter_value().double_value
        coneHeight = self.get_parameter("/physical/cone_height").get_parameter_value().double_value
        minPoints = self.get_parameter("/perception/lidar/cone_filter/min_points").get_parameter_value().integer_value
        l2Th = self.get_parameter("/perception/lidar/cone_filter/l2_th").get_parameter_value().double_value
        linTh = self.get_parameter("/perception/lidar/cone_filter/lin_th").get_parameter_value().double_value

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
        clusterStrategy = self.get_parameter("/perception/lidar/cluster_strategy").get_parameter_value().string_value

        if clusterStrategy == "mean_shift":
            nGridCellsX = self.get_parameter("/perception/lidar/mean_shift/n_grid_cells_x").get_parameter_value().integer_value
            nGridCellsY = self.get_parameter("/perception/lidar/mean_shift/n_grid_cells_y").get_parameter_value().integer_value
            nGridCells = [nGridCellsX, nGridCellsY]
            nmsRadius = self.get_parameter("/perception/lidar/mean_shift/nms_radius").get_parameter_value().double_value
            nIters = self.get_parameter("/perception/lidar/mean_shift/n_iters").get_parameter_value().integer_value
            minClusterPoints = self.get_parameter("/perception/lidar/mean_shift/min_cluster_points").get_parameter_value().integer_value
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
        coneRadius = self.get_parameter("/physical/cone_radius").get_parameter_value().double_value
        coneHeight = self.get_parameter("/physical/cone_height").get_parameter_value().double_value
        lidarHeight = self.get_parameter("/physical/lidar_height").get_parameter_value().double_value
        markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)
        return markerViz
    
    def timer_callback(self):
        try:
            out = self.lidar.run(self)
        except Exception as exp:  # pylint: disable=broad-except
            self.get_logger().warn("Lidar Pipeline failed: " + str(exp))
            return
        if out is None:
            return
        # self.status.running()
        



def main(args = None) -> None:
    """
    Main Loop
    """
    # print in the console
    print("Starting Lidar Node")
    rclpy.init(args = args)
    node = LidarSystem()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
