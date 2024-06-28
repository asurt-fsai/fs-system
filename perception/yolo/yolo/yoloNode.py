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
from asurt_msgs.msg import BoundingBoxes, ConeImgArray
from yolo.helpers import processBboxes, cropBboxes

# mypy: disable-error-code="misc"
class YoloNode(Node):

    def __init__(self) -> None:

        super.__init__("yoloNode")
        self.get_logger().info("STARTING YOLOV8 NODE")

        self.frameId: str
        self.view_id: int

        self.bridge: CvBridge
        self.detector: YOLO

        self.dets_pub: rclpy.publisher.Publisher
        self.crpd_dets_pub: rclpy.publisher.Publisher

        self.proc_bboxes: BoundingBoxes
        self.crpd_bboxes: ConeImgArray

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
        """
        Get parameters from the parameter server and set them to their respective variables
        initalized CvBridge & detector
        """
        
        self.frameId = self.get_parameter("perception.yolo.frame_id").get_parameter_value().string_value
        
        model_path = self.get_parameter('model.model_path').get_parameter_value().string_value
        self.detector = YOLO(model_path)
        self.bridge = CvBridge()
        

    def initPubAndSub(self) -> None:
        """
        Initialize Publishers and subscribers for yolo node
        """
        dets_topic = self.get_parameter("perception.yolo.detections").get_parameter_value().string_value
        crpd_dets_topic = self.get_parameter("perception.yolo.cropped_detections").get_parameter_value().string_value
        camera_feed_topic = self.get_parameter("perception.yolo.camera_feed").get_parameter_value().string_value

        self.dets_pub = self.create_publisher(BoundingBoxes, dets_topic, 1)
        self.crpd_dets_pub = self.create_publisher(ConeImgArray, crpd_dets_topic, 1)
        self.create_subscription(Image, camera_feed_topic, self.callback_yolo, 1) 
    
    def callback_yolo(self, msg: Image) -> None:
        """
        Callback function to process incoming image messages using YOLO object detection.
        Publishes detected cones' bounding boxes & cropped images.

        Args:
            msg (sensor_msgs.msg.Image): ROS Image message containing the input image to be processed.
        Raises:
            Exception: If any error occurs during image processing or publishing, it logs the error.

        """
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Process the image through YOLO
        try:
            result = self.detector(cv_image)[0]
            
            if (len(result)>0):
                boxes = result.boxes.cpu().numpy()

                # Process bounding boxes
                self.proc_bboxes = processBboxes(boxes, msg.header.frame_id)
                
                # Crop bounding boxes
                self.crpd_bboxes = cropBboxes(cv_image, boxes, msg.header.frame_id, self.bridge)

                self.view_id = msg.header.frame_id

            else:
                self.get_logger().info("Image has no detected objects")

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}') 

    def run(self) -> None:
        """
        Runs the yolo node and publishes the results if there is any
        """
        if self.proc_bboxes or self.crpd_bboxes is None:
            return None
        
        # Publish bounding boxes
        self.dets_pub.publish(self.proc_bboxes)

        # Publish cropped bounding boxes
        self.crpd_dets_pub.publish(self.crpd_bboxes) 

        self.get_logger().info('Published processed results' + self.view_id)

        # Reset results
        self.proc_bboxes = None
        self.crpd_bboxes = None


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