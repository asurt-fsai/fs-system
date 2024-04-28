#!/usr/bin/env python3

import rclpy
import cv2
import sys
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):

    def __init__(self):

        super().__init__("camera_interface_node")
        self.get_logger().info("STARTING CAMERA NODE...")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/camera_interface/rate', rclpy.Parameter.Type.INTEGER),
                ('/camera_interface/image_width', rclpy.Parameter.Type.INTEGER),
                ('/camera_interface/image_height', rclpy.Parameter.Type.INTEGER),
                ('/camera_interface/cx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/cy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k3', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/camera_feed', rclpy.Parameter.Type.STRING)
            ]
        )

        self.timer = None
        self.img_publisher = None

        self.bridge = None
        self.cap = None
        self.frame_id = 0

        #undistortion maps
        self.mapx = None
        self.mapy = None 

        self.start()
    
    def get_params(self):
        try:
            # Video feed params
            rate = self.get_parameter('/camera_interface/rate').get_parameter_value().integer_value  # Hz/fps
            width = self.get_parameter('/camera_interface/image_width').get_parameter_value().integer_value
            height = self.get_parameter('/camera_interface/image_height').get_parameter_value().integer_value
            # Camera intrinsics
            cx = self.get_parameter('/camera_interface/cx').get_parameter_value().double_value
            cy = self.get_parameter('/camera_interface/cy').get_parameter_value().double_value
            fx = self.get_parameter('/camera_interface/fx').get_parameter_value().double_value
            fy = self.get_parameter('/camera_interface/fy').get_parameter_value().double_value
            # Distortion coefficients
            k1 = self.get_parameter('/camera_interface/k1').get_parameter_value().double_value
            k2 = self.get_parameter('/camera_interface/k2').get_parameter_value().double_value
            p1 = self.get_parameter('/camera_interface/p1').get_parameter_value().double_value
            p2 = self.get_parameter('/camera_interface/p2').get_parameter_value().double_value
            k3 = self.get_parameter('/camera_interface/k3').get_parameter_value().double_value

            # Constructing and returning the params dictionary
            params = {
                'rate': rate,
                'width': width,
                'height': height,
                'cx': cx,
                'cy': cy,
                'fx': fx,
                'fy': fy,
                'k1': k1,
                'k2': k2,
                'p1': p1,
                'p2': p2,
                'k3': k3
            }

            # Printing fetched parameters for confirmation
            self.get_logger().info(f"Fetched parameters successfully: {params}")
            return params

        except Exception as e:
            # Logging the error and re-raising
            self.get_logger().error(f"Failed to fetch parameters: {e}")

    def generate_undistort_maps(self, params):

        camera_matrix = np.array([[params['fx'], 0, params['cx']], [0, params['fy'], params['cy']], [0, 0, 1]]) 
        dist_coeffs = np.array([[params['k1']], [params['k2']], [params['p1']], [params['p2']], [params['k3']]])

        # Generate undistortion maps
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, (params['width'], params['height']), cv2.CV_32FC1)
        

    def start(self):

        #fetch parameters from launch file
        params = self.get_params()

        #define timer and publisher
        self.timer = self.create_timer(1/params['rate'], self.callback_camera)
        camera_feed_topic = self.get_parameter('/camera_interface/camera_feed').get_parameter_value().string_value
        self.img_publisher = self.create_publisher(Image, camera_feed_topic, 10)

        #define CVbridge
        self.bridge = CvBridge()

        #create undistortion maps
        self.generate_undistort_maps(params)

        #set-up camera capture 
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            sys.exit(1)

        #set camera capture resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, params['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, params['height'])
        
        #set camera capture FPS
        self.cap.set(cv2.CAP_PROP_FPS, params['rate'])
        

    def callback_camera(self):

        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Error capturing image')
                return
            
            # extract the left image from the ZED's stereo pair
            left_img = np.split(frame, 2, axis=1)[0]

            #undistort
            undistorted_img = cv2.remap(left_img, self.mapx, self.mapy, cv2.INTER_LINEAR)
            
            # Convert OpenCV image to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(undistorted_img, encoding="rgb8")
            ros_image.header.frame_id = str(self.frame_id)
            
            # Publish the image
            self.img_publisher.publish(ros_image)

            self.get_logger().info('Publishing camera feed frame: ' + str(self.frame_id))
            self.frame_id += 1
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def __del__(self):
        self.cap.release()

def main(args=None):

    rclpy.init(args=args)
    node = CameraNode()
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