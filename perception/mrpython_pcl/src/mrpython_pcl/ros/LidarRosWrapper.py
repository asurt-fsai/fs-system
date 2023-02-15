"""
A ros wrapper for the LidarPipeline class
"""
from typing import Dict, Any, Optional

import rospy
import numpy as np
import numpy.typing as npt
from sensor_msgs.msg import PointCloud2
from tf_helper.MarkerViz import MarkerViz

from ..LidarPipeline.LidarPipeline import LidarPipeline
from .Serializers import npToRos, npConesToRos, rosToPcl


class LidarRosWrapper(LidarPipeline):
    """
    A ros wrapper for the lidar pipeline class
    It allows the class to read sensor_msgs.msg.PointCloud2 point cloud
    It also publishes the detected cones to the publishers given in the initialization
    """

    def __init__(
        self,
        publishers: Dict[str, rospy.Publisher],
        markerViz: MarkerViz,
        *args: Any,
        **kwargs: Any,
    ):
        super().__init__(*args, **kwargs)
        self.publishers = publishers
        self.markerViz = markerViz

        # Parameter Validation
        try:
            for key, value in publishers.items():
                assert isinstance(value, rospy.Publisher)

            for key in ["filtered", "clustered", "detected"]:
                assert key in publishers
        except Exception as exp:
            errMsg = "LidarRosWrapper: ensure the all the publishers required are provided \n \
                       - filtered \n\
                       - clustered \n\
                       - detected \n\
                       - optional: tracked"
            raise TypeError(errMsg) from exp

    def setPointcloud(self, pointcloud: PointCloud2) -> None:
        pointcloud = rosToPcl(pointcloud)
        super().setPointcloud(pointcloud)

    def run(self) -> Optional[Dict[str, npt.NDArray[np.float64]]]:
        output: Optional[Dict[str, npt.NDArray[np.float64]]] = super().run()
        if output is None:
            return None

        output["filtered"] = npToRos(output["filtered"])
        output["clustered"] = npToRos(output["clustered"])
        output["clusterCenters"] = npConesToRos(output["clusterCenters"])
        output["detected"] = npConesToRos(output["detected"])

        self.publishers["filtered"].publish(output["filtered"])
        self.publishers["clustered"].publish(output["clustered"])
        self.publishers["detected"].publish(output["detected"])

        # MarkerArray visualizations
        detectedMarkers = self.markerViz.conesToMarkers(output["detected"])
        self.publishers["detected_markers"].publish(detectedMarkers)

        if "tracked" in output:
            output["tracked"] = npConesToRos(output["tracked"], addIDs=True)
            self.publishers["tracked"].publish(output["tracked"])

            trackedMarkers = self.markerViz.conesToMarkers(output["tracked"])
            self.publishers["tracked_markers"].publish(trackedMarkers)

        return output
