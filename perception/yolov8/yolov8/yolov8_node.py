#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from my_msgs import BoundingBoxes
from utils import processBboxes

class Yolov8Node(Node):

    def __init__(self):
        
        super().__init__("yolov8_node")
        self.get_logger().info("STARTING YOLOV8 NODE")

        self.img_sub = self.create_subscription(Image, "/camera_interface/camera_feed", self.callback_yolo, 10)
        self.res_pub = self.create_publisher(BoundingBoxes, "/yolov8/detections", 10)

        self.bridge = CvBridge()
        
        self.detector = YOLO("./best.pt")

    def callback_yolo(self, msg: Image):

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rbg8")
            # Process the image through YOLO
            results = self.model(cv_image)
            # Process bounding boxes
            processed_results = processBboxes(results, msg.header.frame_id)
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