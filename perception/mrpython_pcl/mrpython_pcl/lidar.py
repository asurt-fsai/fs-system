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
from tf_helper.MarkerViz import MarkerViz
# from smoreo.MarkerViz import MarkerViz

from .LidarPipeline.Filter.Filter import Filter
from .LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from .LidarPipeline.ConeClassifier.ConeClassifier import ConeClassifier
from .LidarPipeline.Clusterer.MeanClusterer import MeanClusterer
from .LidarPipeline.Clusterer.AbstractClusterer import Clusterer
from .LidarRosWrapper import LidarRosWrapper
from tf_helper.StatusPublisher import StatusPublisher

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
                ('/perception/lidar/view_bounds/zmax', rclpy.Parameter.Type.DOUBLE),

                ('/perception/lidar/car_bounds/xmin', rclpy.Parameter.Type.INTEGER),
                ('/perception/lidar/car_bounds/xmax', rclpy.Parameter.Type.DOUBLE),
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
        self.status = StatusPublisher("/status/lidar",self)
        self.status.starting()
        

        self.lidar = self.buildPipeline()
        self.status.ready()

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
            return self.get_parameter_or(param, default).get_parameter_value()
        return self.get_parameter(param).get_parameter_value()

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
        lidarHeight = self.getParam("/physical/lidar_height",0.1).double_value
        subsample = self.getParam("/perception/lidar/subsample",False).bool_value  
        
        publishers_filtered, publishers_clustered,publishers_detected, publishers_tracked,publishers_detected_markers= self.buildPublishers()
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

        velodyneTopic = self.getParam("/perception/lidar/velodyne_topic","/velodyne_points").string_value
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
        filteredTopic = self.getParam("/perception/lidar/filtered_topic","/placeholder/lidar/filtered").string_value
        clusteredTopics = self.getParam("/perception/lidar/clustered_topic", "/placeholder/lidar/clustered").string_value
        detectedConesTopic = self.getParam("/perception/lidar/detected_topic", "/placeholder/lidar/detected").string_value
        trackedConesTopic = self.getParam("/perception/lidar/tracked_topic", "/placeholder/lidar/tracked").string_value
        detectedMarkersTopic = self.getParam("/perception/lidar/detected_markers_topic", "/placeholder/lidar/detected_markers").string_value

     
        self.publishers_filtered = self.create_publisher( PointCloud2,filteredTopic, 10)
        self.publishers_clustered = self.create_publisher(PointCloud2,clusteredTopics,  10)
        self.publishers_detected = self.create_publisher( LandmarkArray,detectedConesTopic, 10)
        self.publishers_tracked = self.create_publisher( LandmarkArray, trackedConesTopic,10)
        self.publishers_detected_markers = self.create_publisher(MarkerArray,detectedMarkersTopic,  10)

        return self.publishers_filtered, self.publishers_clustered,  self.publishers_detected, self.publishers_tracked,  self.publishers_detected_markers

 



    def buildFilter(self) -> Filter:
        """
        Creates a filter along with its ground removal method using the ros parameters set

        Returns
        -------
        Filter
            The created Filter object
        """
        ransacThreshold = self.getParam("/perception/lidar/ransac_threshold",0.1).double_value
        lidarHeight = self.getParam("/physical/lidar_height",0.1).double_value

        viewBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0], "z": [0.0, 0.0]}
        viewBounds["x"][0] = self.getParam("/perception/lidar/view_bounds/xmin",-10).integer_value
        viewBounds["x"][1] = self.getParam("/perception/lidar/view_bounds/xmax",10).integer_value

        viewBounds["y"][0] = self.getParam("/perception/lidar/view_bounds/ymin",-6).integer_value
        viewBounds["y"][1] = self.getParam("/perception/lidar/view_bounds/ymax",6).integer_value

        viewBounds["z"][0] = -1 * lidarHeight + ransacThreshold
        viewBounds["z"][1] = self.getParam("/perception/lidar/view_bounds/zmax",2.0).double_value

        carBounds = {"x": [0.0, 0.0], "y": [0.0, 0.0]}

        carBounds["x"][0] = self.getParam("/perception/lidar/car_bounds/xmin",-2).integer_value
        carBounds["x"][1] = self.getParam("/perception/lidar/car_bounds/xmax",0.0).double_value  

        carBounds["y"][0] = self.getParam("/perception/lidar/car_bounds/ymin",-0.75).double_value
        carBounds["y"][1] = self.getParam("/perception/lidar/car_bounds/ymax",0.75).double_value

        reconstructParam = self.getParam("/perception/lidar/reconst_radius",0.228).double_value    

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
        coneRadius = self.getParam("/physical/cone_radius",0.228).double_value
        coneHeight = self.getParam("/physical/cone_height",0.4).double_value
        minPoints = self.getParam("/perception/lidar/cone_filter/min_points",5).integer_value
        l2Th = self.getParam("/perception/lidar/cone_filter/l2_th",0.03).double_value
        linTh = self.getParam("/perception/lidar/cone_filter/lin_th",1e-4).double_value

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
        clusterStrategy = self.getParam("/perception/lidar/cluster_strategy","mean_shift").string_value

        if clusterStrategy == "mean_shift":
            nGridCellsX = self.getParam("/perception/lidar/mean_shift/n_grid_cells_x",40).integer_value
            nGridCellsY = self.getParam("/perception/lidar/mean_shift/n_grid_cells_y",40).integer_value
            nGridCells = [nGridCellsX, nGridCellsY]
            nmsRadius = self.getParam("/perception/lidar/mean_shift/nms_radius",0.4).double_value
            nIters = self.getParam("/perception/lidar/mean_shift/n_iters",3).integer_value
            minClusterPoints = self.getParam("/perception/lidar/mean_shift/min_cluster_points",5).integer_value
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
        coneRadius = self.getParam("/physical/cone_radius",0.228).double_value
        coneHeight = self.getParam("/physical/cone_height",0.4).double_value
        lidarHeight = self.getParam("/physical/lidar_height",0.1).double_value
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
        self.status.running()
        



def main(args = None) -> None:
    """
    Main Loop
    """    
   
    rclpy.init(args = args)
    node = LidarSystem()
    node.get_logger().info("Starting Lidar Node")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
