"""
A ros wrapper for the LidarPipeline class
"""
from typing import Dict, Any, Optional

import numpy as np
import numpy.typing as npt
from sensor_msgs.msg import PointCloud2
from tf_helper.MarkerViz import MarkerViz
import rclpy

from .LidarPipeline.LidarPipeline import LidarPipeline
from .Serializers import npToRos, npConesToRos, rosToPcl


class LidarRosWrapper(LidarPipeline):
    """
    A ros wrapper for the lidar pipeline class
    It allows the class to read sensor_msgs.msg.PointCloud2 point cloud
    It also publishes the detected cones to the publishers given in the initialization
    """

    def __init__(
        self,
        markerViz: MarkerViz,
        publishersFiltered: rclpy.publisher.Publisher,
        publishersClustered: rclpy.publisher.Publisher,
        publishersDetected: rclpy.publisher.Publisher,
        publishersTracked: rclpy.publisher.Publisher,
        publishersDetectedMarkers: rclpy.publisher.Publisher,
        node,
        *args: Any,
        **kwargs: Any,
    ):
        super().__init__(*args, **kwargs)
        self.markerViz = markerViz
        self.frameId = ""
        self.node = node

        # Parameter Validation
        publishers = [
            publishersFiltered,
            publishersClustered,
            publishersDetected,
            publishersTracked,
            publishersDetectedMarkers,
        ]

        try:
            for value in publishers:
                assert isinstance(value, rclpy.publisher.Publisher)

        except Exception as exp:
            errMsg = "LidarRosWrapper: ensure the all the publishers required are provided \n \
                       - filtered \n\
                       - clustered \n\
                       - detected \n\
                       - optional: tracked"
            raise TypeError(errMsg) from exp

    def setPointcloud(self, pointcloud: PointCloud2) -> None:
        """
        This method receives a ROS PointCloud2 message, converts it to a PCL format,
        sets the point cloud data for further processing.
        Args:
            pointcloud (PointCloud2): The input point cloud data in ROS PointCloud2 format.

        Returns:
            None
        """
        self.frameId = pointcloud.header.frame_id
        pointcloud = rosToPcl(pointcloud)
        super().setPointcloud(pointcloud)
        self.node.get_logger().info("Received pointcloud")

    def run(self, node) -> Optional[Dict[str, npt.NDArray[np.float64]]]:
        """

        This method runs the point cloud processing pipeline, converting processed
        numpy arrays back to ROS messages, publishing various intermediate and final
        results, and generating visualization markers.

        Args:
            node: The ROS node that contains the publishers for various topics.

        Returns:
            Optional[Dict[str, npt.NDArray[np.float64]]]: A dictionary containing the processed
            data, with keys such as 'filtered', 'clustered', 'clusterCenters', 'detected', and
            optionally 'tracked'. Each key maps to a numpy array of processed data.
        """
        output: Optional[Dict[str, npt.NDArray[np.float64]]] = super().run()

        if output is None:
            return None

        output["filtered"] = npToRos(output["filtered"], self.frameId)
        output["clustered"] = npToRos(output["clustered"], self.frameId)

        output["clusterCenters"] = npConesToRos(output["clusterCenters"], self.frameId)
        output["detected"] = npConesToRos(output["detected"], self.frameId)

        node.publishersFiltered.publish(output["filtered"])
        node.publishersClustered.publish(output["clustered"])
        node.publishersDetected.publish(output["detected"])

        # MarkerArray visualizations
        detectedMarkers = self.markerViz.conesToMarkers(output["detected"])
        node.publishersDetectedMarkers.publish(detectedMarkers)

        if "tracked" in output:
            output["tracked"] = npConesToRos(output["tracked"], self.frameId, addIDs=True)

            node.publishers["tracked"].publish(output["tracked"])

            trackedMarkers = self.markerViz.conesToMarkers(output["tracked"])

            node.publishersTrackedMarkers.publish(trackedMarkers)

        return output
