#!/usr/bin/env python3
import rclpy
import os

from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from asurt_msgs.msg import BoundingBoxes

class Yolov8Node(Node):

    def __init__(self):
        
        super().__init__("yolov8_node")
        self.get_logger().info("STARTING YOLOV8 NODE")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/camera_interface/camera_feed', rclpy.Parameter.Type.STRING),
                ('/yolov8/detections', rclpy.Parameter.Type.STRING),
                ('/yolov8/model_path', rclpy.Parameter.Type.STRING)
            ]
        )

        self.img_sub = None
        self.res_pub = None

        self.bridge = None
        
        self.detector = None

        self.start()

    def start(self):

        #fetch parameters from launch file
        camera_feed_topic = self.get_parameter('/camera_interface/camera_feed').get_parameter_value().string_value
        detections_topic = self.get_parameter('/yolov8/detections').get_parameter_value().string_value
        MODEL_PATH = self.get_parameter('/yolov8/model_path').get_parameter_value().string_value

        #define subscriber & publisher
        self.img_sub = self.create_subscription(Image, camera_feed_topic, self.callback_yolo, 10)
        self.res_pub = self.create_publisher(BoundingBoxes, detections_topic, 10)

        #define CvBridge
        self.bridge = CvBridge()
        #define model
        self.detector = YOLO(MODEL_PATH)

    def processBboxes(self):
        pass

    def callback_yolo(self, msg: Image):

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            # Process the image through YOLO
            results = self.model(cv_image)
            # Process bounding boxes
            processed_results = self.processBboxes(results, msg.header.frame_id)
            # Publish the results
            self.res_pub.publish(processed_results)

            self.get_logger().info('Published processed results' + msg.header.frame_id)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = Yolov8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down.")
    except Exception as e:
        print(f"Caught exception: {e}") # Handle other exceptions
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()