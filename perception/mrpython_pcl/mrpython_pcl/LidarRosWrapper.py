"""
A ros wrapper for the LidarPipeline class
"""
from typing import Dict, Any, Optional

import rclpy
import numpy as np
import numpy.typing as npt
from sensor_msgs.msg import PointCloud2
from tf_helper.MarkerViz import MarkerViz
# from smoreo.MarkerViz import MarkerViz


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
        publishers_filtered: rclpy.publisher.Publisher,
        publishers_clustered: rclpy.publisher.Publisher,
        publishers_detected: rclpy.publisher.Publisher,
        publishers_tracked: rclpy.publisher.Publisher,
        publishers_detected_markers: rclpy.publisher.Publisher,
        *args: Any,
        **kwargs: Any,
    ):
        super().__init__(*args, **kwargs)
        self.markerViz = markerViz
        self.frameId = ""

        # Parameter Validation
        publishers = [publishers_filtered , publishers_clustered, 
                      publishers_detected, publishers_tracked,    
                      publishers_detected_markers]

        try:
            for value in publishers:
                assert isinstance(value, rclpy.publisher.Publisher)

            # for key in ["filtered", "clustered", "detected"]:
            #     assert key in publishers
        except Exception as exp:
            errMsg = "LidarRosWrapper: ensure the all the publishers required are provided \n \
                       - filtered \n\
                       - clustered \n\
                       - detected \n\
                       - optional: tracked"
            raise TypeError(errMsg) from exp

    def setPointcloud(self, pointcloud: PointCloud2) -> None:
        self.frameId = pointcloud.header.frame_id
        pointcloud = rosToPcl(pointcloud)
        super().setPointcloud(pointcloud)

    def run(self,node) -> Optional[Dict[str, npt.NDArray[np.float64]]]:
        output: Optional[Dict[str, npt.NDArray[np.float64]]] = super().run()

        if output is None:
            return None

        output["filtered"] = npToRos(output["filtered"], self.frameId)
        output["clustered"] = npToRos(output["clustered"], self.frameId)
        output["clusterCenters"] = npConesToRos(output["clusterCenters"], self.frameId)
        output["detected"] = npConesToRos(output["detected"], self.frameId)

        node.publishers_filtered.publish(output["filtered"])
        node.publishers_clustered.publish(output["clustered"])
        node.publishers_detected.publish(output["detected"])

        # MarkerArray visualizations
        detectedMarkers = self.markerViz.conesToMarkers(output["detected"])
        node.publishers_detected_markers.publish(detectedMarkers)

        if "tracked" in output:
            output["tracked"] = npConesToRos(output["tracked"], self.frameId, addIDs=True)
            node.publishers["tracked"].publish(output["tracked"])

            trackedMarkers = self.markerViz.conesToMarkers(output["tracked"])
            node.publishers_tracked_markers.publish(trackedMarkers)

        return output
