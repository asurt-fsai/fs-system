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

        #node parameters: should be moved to launch file and fetched here
        #video feed params
        self.RATE = 30 # Hz/fps
        self.WIDTH = 640
        self.HEIGHT = 480
        #camera intrinsics
        cx = 0
        cy = 0
        fx = 0
        fy = 0
        #distortion coeff.
        k1 = 0
        k2 = 0
        p1 = 0
        p2 = 0
        k3 = 0

        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]) 
        self.dist_coeffs = np.array([[k1], [k2], [p1], [p2], [k3]])

        # Generate undistortion maps
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, None, (self.WIDTH, self.HEIGHT), cv2.CV_32FC1)

        self.timer = self.create_timer(1/self.RATE, self.callback_camera)
        self.img_pub = self.create_publisher(Image, "/camera_interface/camera_feed", 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.frame_id = 0

        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        # Set the capture FPS 
        self.cap.set(cv2.CAP_PROP_FPS, self.RATE)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            sys.exit(1)

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
            self.img_pub.publish(ros_image)

            self.get_logger().info('Publishing webcam image' + str(self.frame_id))
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