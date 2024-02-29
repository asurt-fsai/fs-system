#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from asurt_msgs.msg import BoundingBoxes, BoundingBox, ConeImg, ConeImgArray

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
                ('/yolov8/cropped_detections', rclpy.Parameter.Type.STRING),
                ('/yolov8/model_path', rclpy.Parameter.Type.STRING)
            ]
        )

        self.img_subscriber = None
        self.bboxes_publisher = None
        self.cropped_bboxes_publisher = None

        self.bridge = None
        
        self.detector = None

        self.start()

    def start(self):

        #fetch parameters from launch file
        camera_feed_topic = self.get_parameter('/camera_interface/camera_feed').get_parameter_value().string_value
        detections_topic = self.get_parameter('/yolov8/detections').get_parameter_value().string_value
        cropped_detections_topic = self.get_parameter('/yolov8/cropped_detections').get_parameter_value().string_value

        MODEL_PATH = self.get_parameter('/yolov8/model_path').get_parameter_value().string_value

        #define subscriber & publisher
        self.img_subscriber = self.create_subscription(Image, camera_feed_topic, self.callback_yolo, 10)
        self.bboxes_publisher = self.create_publisher(BoundingBoxes, detections_topic, 10)
        self.cropped_bboxes_publisher = self.create_publisher(ConeImgArray, cropped_detections_topic, 10) 

        #define CvBridge
        self.bridge = CvBridge()
        #define model
        self.detector = YOLO(MODEL_PATH)

    def processBboxes(self, boxes, frame_id):
        
        detections = BoundingBoxes()

        detections.frame_id = frame_id
        detections.object_count = len(boxes)
        detections.bounding_boxes = [None] * detections.object_count

        for idx, box in enumerate(boxes):
            
            detection = BoundingBox()

            detection.probability = box.conf[0]

            detection.xmin, detection.ymin, detection.xmax, detection.ymax = box.xyxy[0]
            detection.x_center, detection.y_center, detection.width, detection.height = box.xywh[0]

            detection.id = idx
            detection.type = box.cls[0]

            detections.bounding_boxes[idx] = detection
        
        return detections

    def cropBboxes(self, img, boxes, frame_id):
        
        cropped_detections = ConeImgArray()

        cropped_detections.frame_id = frame_id
        cropped_detections.object_count = len(boxes)
        cropped_detections.imgs = [None] * cropped_detections.object_count

        for idx, box in enumerate(boxes):

            cropped_detection = ConeImg()

            cropped_detection.id = idx
            cropped_detection.rows = box.shape[0]
            cropped_detection.cols = box.shape[1]

            xmin, ymin, xmax, ymax = box.xyxy[0]
            cropped_detection.data = img[ymin:ymax, xmin:xmax].ravel()

            cropped_detections.imgs[idx] = cropped_detection

        return cropped_detections

    def callback_yolo(self, msg: Image):

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Process the image through YOLO
            result = self.model(cv_image)
            result = result.boxes.cpu().numpy()

            # Process bounding boxes
            processed_results = self.processBboxes(result, msg.header.frame_id)

            # Crop bounding boxes
            cropped_bboxes = self.cropBboxes(cv_image, result, msg.header.frame_id)

            # Publish bounding boxes
            self.bboxes_publisher.publish(processed_results)

            # Publish cropped bounding boxes
            self.cropped_bboxes_publisher.publish(cropped_bboxes) 

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