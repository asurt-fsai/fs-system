#!/usr/bin/python3
"""
Main ros node for the smoreo pipeline used to detect cone
"""
# from tf_helper.StatusPublisher import StatusPublisher
import rclpy
from rclpy.node import Node
from typing import Dict, Any, Union
import numpy as np
# import tf
# from tf_transformations import quaternion_matrix
import transforms3d as tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray
from smoreo.utils import processBboxes
from asurt_msgs.msg import BoundingBoxes
# from tf_helper.MarkerViz import MarkerViz
from smoreo.MarkerViz import MarkerViz

from smoreo.smoreo import Smoreo


class SmoreoSystem(Node):
    def __init__(self):
        super().__init__("smoreo")
        self.create_timer(0.01, self.timer_callback)
        # self.status = StatusPublisher("/status/smoreo")
        
        self.params: Dict[str, Any]
        self._publishers_landmarkPub: rclpy.publisher.Publisher
        self._publishers_markersPub: rclpy.publisher.Publisher
        self.boundingBoxes: BoundingBoxes
        self.smoreo: Smoreo
        self.useConeBase: bool
        self.inTuning: bool
        self.markerViz: MarkerViz

        self.declare_parameters(
            namespace='',
            parameters=[
                ('/smoreo/use_cone_base', rclpy.Parameter.Type.BOOL),
                ('/smoreo/in_tuning', rclpy.Parameter.Type.BOOL),
                ('/smoreo/cut_off_y', rclpy.Parameter.Type.INTEGER),
                ('/physical/camera_height_from_ground', rclpy.Parameter.Type.DOUBLE),
                ('/physical/cone_height', rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/camera_info', rclpy.Parameter.Type.STRING),
                ('/smoreo/hardcode_params', rclpy.Parameter.Type.BOOL),
                ('/smoreo/cx', rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/cy', rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/f', rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/predicted_landmarks', rclpy.Parameter.Type.STRING),
                ('/smoreo/predicted_markers', rclpy.Parameter.Type.STRING),
                ('/smoreo/bounding_boxes', rclpy.Parameter.Type.STRING),
                ('/physical/cone_radius', rclpy.Parameter.Type.DOUBLE),
                ('/physical/lidar_height', rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/camera_frame', rclpy.Parameter.Type.STRING)
            ]
            )

        self.cone_base = self.get_parameter("/smoreo/use_cone_base").get_parameter_value().bool_value
        self.in_tuning = self.get_parameter("/smoreo/in_tuning").get_parameter_value().bool_value
    def getParams(self) -> Dict[str, Any]:
        """
        Get params of the system from the ros parameters server

        Parameters:
        ------------
        None
        Returns:
        -----------
        Dict[str,Any]: parameters obtained from the ros parameter server.
        """
        try:
            for key in [
                "/smoreo/cut_off_y",
                "/physical/camera_height_from_ground",
                "/physical/cone_height",
                "/smoreo/camera_info",
                "/smoreo/hardcode_params",
            ]:
                assert self.has_parameter(key)

            if self.get_parameter("/smoreo/hardcode_params").get_parameter_value().bool_value:
                assert self.has_parameter("/smoreo/cx")
                assert self.has_parameter("/smoreo/cy")
                assert self.has_parameter("/smoreo/f")
        except Exception as exp:
        
            errMsg = "smoreo: ensure all the required parameters are provided in ros server\n \
                       - cut_off_y \n\
                       - camera_height_from_ground \n\
                       - cone_height \n\
                       - camera_info \n\
                       - hardcode_params \n\
                        - cx \n\
                        - cy \n\
                        - f"
            raise TypeError(errMsg) from exp
        if self.get_parameter("/smoreo/hardcode_params").get_parameter_value().bool_value:
            fInPixels = self.get_parameter("/smoreo/f").get_parameter_value().double_value
            cameraCx = self.get_parameter("/smoreo/cx").get_parameter_value().double_value
            camerCy = self.get_parameter("/smoreo/cy").get_parameter_value().double_value
        else:
            cameraInfo = rclpy.wait_for_message(self.get_parameter("/smoreo/camera_info").get_parameter_value.string_value, CameraInfo)
            fInPixels = cameraInfo.K[0]
            cameraCx = cameraInfo.K[2]
            camerCy = cameraInfo.K[5]

        # coneHeight = self.get_parameter("/physical/cone_height", 0.4).get_parameter_value().double_value
        coneHeight = self.get_parameter("/physical/cone_height").get_parameter_value().double_value

        cutOffY = self.get_parameter("/smoreo/cut_off_y").get_parameter_value().double_value

        worldToCameraRotation = [0.5, -0.5, 0.5, 0.5]
        params = {
            "cx": cameraCx,
            "cy": camerCy,
            "f": fInPixels,
            "k": np.array(
                [[fInPixels, 0.0, cameraCx], [0.0, fInPixels, camerCy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            # "worldCords_inCamera": np.array(
            #     tf.transformations.quaternion_matrix(worldToCameraRotation)[:3, :3], dtype=int
            # ),
            "worldCords_inCamera": np.array(
            tf.quaternions.quat2mat(worldToCameraRotation)[:3, :3], dtype=int
            ),
            "camera_height_from_ground": self.get_parameter("/physical/camera_height_from_ground").get_parameter_value().double_value,
            "cut_off_y": float(cutOffY),
            "cone_height": coneHeight,
        }
        return params

    def createPublishers(self) -> None:
        """
        Create needed publishers, for now only for the predicted landmarks, and the marker array.
        """
        try:
            assert self.has_parameter("/smoreo/predicted_landmarks")
            assert self.has_parameter("/smoreo/predicted_markers")
        except Exception as exp:
            errMsg = "smoreo: ensure all the required topics for\n \
                         publishing are provided in ros server\n \
                        - predicted_landmarks\n \
                        - predicted_markers"
            raise ValueError(errMsg) from exp

        self._publishers_landmarkPub = self.create_publisher(LandmarkArray,
            self.get_parameter("/smoreo/predicted_landmarks").get_parameter_value().string_value, 10
        )

        self._publishers_markersPub = self.create_publisher(MarkerArray,
            self.get_parameter("/smoreo/predicted_markers").get_parameter_value().string_value, 10
        )


    def run(self) -> Union[LandmarkArray, None]:
        """
        Run the smoreo system.
        """
        if self.boundingBoxes is not None:
            # Get parameters every run if in tuning mode
            if self.inTuning:
                newParams = self.getParams()
                self.smoreo.updateParams(newParams)
            predictedLandmarks = self.smoreo.predictLandmarks(self.boundingBoxes)
            self.boundingBoxes = None

            try:
                predictedLandmarks.header.stamp = rclpy.clock.Clock.now()
            except rclpy.exceptions.NotInitializedException:
                rclpy.logging.warn("ROS not initialized, using time 0 for header")

            self.publishers_landmarkPub.publish(predictedLandmarks)
            self.publishers_markersPub.publish(self.markerViz.conesToMarkers(predictedLandmarks))
            return predictedLandmarks
        return None
    
    def setBoundingBoxes(self, boundingBoxes: BoundingBoxes) -> None:
        """
        Main callback function for the system that accepts bounding boxes from the subscriber.

        Parameters:
        ------------
        boundingBoxes: BoundingBoxes

        Returns:
        -----------
        None
        """
        self.boundingBoxes = processBboxes(boundingBoxes)

    def start(self, useConeBase: bool, inTuning: bool) -> None:
        """
        start the smoreo system by creating publishers, subscribers and the smoreo object.
        """
        self.useConeBase = useConeBase
        self.inTuning = inTuning
        self.createPublishers()
        self.params = self.getParams()
        coneRadius = self.get_parameter("/physical/cone_radius").get_parameter_value().double_value
        coneHeight = self.get_parameter("/physical/cone_height").get_parameter_value().double_value
        lidarHeight = self.get_parameter("/physical/lidar_height").get_parameter_value().double_value
        self.markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)
        self.boundingBoxes = None
        self.smoreo = Smoreo(self.params, self.get_parameter("/smoreo/camera_frame").get_parameter_value().string_value)
        if not self.has_parameter("/smoreo/bounding_boxes"):
            raise ValueError("smoreo: bounding boxes topic not provided")
        self.subscription= self.create_subscription( BoundingBoxes,
            self.get_parameter("/smoreo/bounding_boxes").get_parameter_value().string_value , self.setBoundingBoxes,10
        )

       
    def timer_callback(self):
        out = self.run()
        if out is None:
            # self.status.running()
            return
        # self.status.running()

def main(args = None) -> None:
    """
    Main Loop
    """
    rclpy.init(args = args)
    node = SmoreoSystem()

    # node.status.starting()


    if not node.has_parameter("/smoreo/use_cone_base"):
        raise ValueError("smoreo: use_cone_base is not set")
    if not node.has_parameter("/smoreo/in_tuning"):
        raise ValueError("smoreo: in_tuning is not set")

    node.start(node.cone_base, node.in_tuning)
    # node.status.ready()
    
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
