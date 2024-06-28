#!/usr/bin/python3
"""
Yolo ros node
"""
import rclpy
import numpy as np

from rclpy.node import Node
from tf_helper.StatusPublisher import StatusPublisher
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from asurt_msgs.msg import BoundingBoxes, BoundingBox, ConeImg, ConeImgArray

# mypy: disable-error-code="misc"
class YoloNode(Node):

    def __init__(self) -> None:

        super.__init__("yoloNode")
        self.get_logger().info("STARTING YOLOV8 NODE")

        self.bridge: CvBridge
        self.detector: YOLO

        self.dets_pub: rclpy.publisher.Publisher
        self.crpd_dets_pub: rclpy.publisher.Publisher

        # start sequence 
        self.declareParameters()
        self.setParameters()
        self.initPubAndSub()

    def declareParameters(self) -> None:
        """
        Declare the parameters for the yolo class
        """
        self.declare_parameter("perception.yolo.frame_id", rclpy.Parameter.Type.STRING)
        self.declare_parameter("perception.yolo.camera_feed", rclpy.Parameter.Type.STRING)
        self.declare_parameter("perception.yolo.detections", rclpy.Parameter.Type.STRING)
        self.declare_parameter("perception.yolo.cropped_detections", rclpy.Parameter.Type.STRING)
        self.declare_parameter("model.model_path", rclpy.Parameter.Type.STRING)

    def setParameters(self) -> None:
        pass

    def initPubAndSub(self) -> None:
        pass

    def run(self) -> None:
        pass


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rclpy.init()
    yolo = YoloNode()
    status = StatusPublisher("/status/yolo", yolo)

    status.starting()

    # Publish heartbeat to show the module is ready
    status.ready()

    # Main loop
    rate = yolo.create_rate(100)
    while rclpy.ok():
        rate.sleep()
        out = yolo.run()
        if out is None:
            continue

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    main()