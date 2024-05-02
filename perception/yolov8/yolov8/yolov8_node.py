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

        detections.view_id = int(frame_id)
        detections.object_count = len(boxes)
        detections.bounding_boxes = [] 

        for idx, box in enumerate(boxes):
            
            detection = BoundingBox()
            detection.probability = float(box.conf[0])
            
            detection.xmin = int(box.xyxy[0][0])
            detection.ymin = int(box.xyxy[0][1])
            detection.xmax = int(box.xyxy[0][2])
            detection.ymax = int(box.xyxy[0][3])
            detection.x_center = int(box.xywh[0][0])
            detection.y_center = int(box.xywh[0][1])
            detection.width = int(box.xywh[0][2])
            detection.height = int(box.xywh[0][3])

            detection.detection_id = idx
            detection.track_id = 0 #yet to be tracked - track ids are 1-indexed
            detection.type = int(box.cls[0])

            detections.bounding_boxes.append(detection)  
        
        return detections

    def cropBboxes(self, img, boxes, frame_id):
        
        cropped_detections = ConeImgArray()

        cropped_detections.view_id = int(frame_id)
        cropped_detections.object_count = len(boxes)
        cropped_detections.imgs = []

        for idx, box in enumerate(boxes):

            cropped_detection = ConeImg()

            cropped_detection.id = idx
            cropped_detection.rows = box.shape[0]
            cropped_detection.cols = box.shape[1]

            xmin = int(box.xyxy[0][0])
            ymin = int(box.xyxy[0][1])
            xmax = int(box.xyxy[0][2])
            ymax = int(box.xyxy[0][3])
            
            cropped_img = img[ymin:ymax, xmin:xmax]
            cropped_detection.img = self.bridge.cv2_to_imgmsg(cropped_img, encoding="rgb8")

            cropped_detections.imgs.append(cropped_detection) 

        return cropped_detections

    def callback_yolo(self, msg: Image):

        
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # Process the image through YOLO
        try:
            result = self.detector(cv_image)[0]
            
            if (len(result)>0):
                boxes = result.boxes.cpu().numpy()

                # Process bounding boxes
                processed_results = self.processBboxes(boxes, msg.header.frame_id)
                
                # Crop bounding boxes
                cropped_bboxes = self.cropBboxes(cv_image, boxes, msg.header.frame_id)
                
                # Publish bounding boxes
                self.bboxes_publisher.publish(processed_results)

                # Publish cropped bounding boxes
                self.cropped_bboxes_publisher.publish(cropped_bboxes) 

                self.get_logger().info('Published processed results' + msg.header.frame_id)
            else:
                self.get_logger().info("Image has no detected objects")

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