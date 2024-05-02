#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from asurt_msgs.msg import BoundingBoxes

class VisualizerNode(Node):

    def __init__(self):

        super().__init__("visualizer_node")
        self.get_logger().info("STARTING VISUALIZER NODE")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_feed', rclpy.Parameter.Type.STRING),
                ('bboxes', rclpy.Parameter.Type.STRING),
                ('viz_bboxes', rclpy.Parameter.Type.STRING)
            ]
        )

        self.bridge = None
        self.last_image = None
        self.view_id = None

        self.img_sub = None
        self.bboxes_sub = None
        self.viz_bboxes_pub = None

        self.bboxes_info = {
            0: ("Blue Cone", (255, 0, 0)),     # BGR for blue
            1: ("Yellow Cone", (0, 255, 255)), # BGR for yellow
            2: ("Large Cone", (0, 165, 255)),  # BGR for orange
            3: ("Orange Cone", (0, 165, 255)), # BGR for orange
            4: ("Unknown", (128, 128, 128))    # BGR for grey
        }

        self.start()

    def start(self):

        #fetch parameters from launch file
        camera_feed_topic = self.get_parameter('camera_feed').get_parameter_value().string_value
        bboxes_topic = self.get_parameter('bboxes').get_parameter_value().string_value
        viz_bboxes_topic = self.get_parameter('viz_bboxes').get_parameter_value().string_value

        #define subscriber & publisher
        self.img_sub = self.create_subscription(Image, camera_feed_topic, self.callback_img, 10)
        self.bboxes_sub = self.create_subscription(BoundingBoxes, bboxes_topic, self.callback_bboxes, 10)
        self.viz_bboxes_pub = self.create_publisher(Image, viz_bboxes_topic, 10) 

        #define CvBridge
        self.bridge = CvBridge()

        self.last_img = None

    def callback_img(self, msg:Image):

        self.get_logger().info("Received Image...")        
        self.last_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8') 
        self.view_id = msg.header.frame_id

    def callback_bboxes(self, msg:BoundingBoxes):
        
        try:
            if self.last_image is not None:

                img = self.last_image.copy()
                self.last_image = None

                for box in msg.bounding_boxes:
                    top_left = (box.xmin, box.ymin)
                    bottom_right = (box.xmax, box.ymax)
                    cv2.rectangle(img, top_left, bottom_right, self.bboxes_info[box.type][1], 2)
                    cv2.putText(img, self.bboxes_info[box.type][0] + ": " +str(box.track_id), (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8") #bgr8 for bags with the color space bug, but rgb8 otherwise
                self.viz_bboxes_pub.publish(ros_img)

                self.get_logger().info('Published processed results' + self.view_id)
        except Exception as e:
           self.get_logger().error(f'Failed to process image: {e}')   

def main(args=None):

    rclpy.init(args=args)
    node = VisualizerNode()
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