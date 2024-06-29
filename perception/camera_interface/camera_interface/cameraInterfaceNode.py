#!/usr/bin/python3
"""
Camera Interface ros node
"""
import rclpy
import cv2
import sys
import numpy as np
import threading

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf_helper.StatusPublisher import StatusPublisher

class CameraNode(Node):
    """
    Camera Interface ros node
    """
    def __init__(self) -> None:
        """
        Init function for camera interface node
        """
        super().__init__("cameraInterfaceNode")
        self.get_logger().info("STARTING CAMERA NODE...")

        self.frameId: str
        self.ViewId: int = 0
        self.bridge: CvBridge

        #set-up camera capture 
        self.cap: cv2.VideoCapture

        self.img_pub: rclpy.publisher.Publisher

        # undistortion maps
        self.mapx: np.ndarray
        self.mapy: np.ndarray

        # start sequence
        self.declareParameters()
        self.setParameters()
        self.initPubAndSub()

    def initPubAndSub(self) -> None:
        """
        Initialize Publishers and subscribers for camera interface node
        """
        camera_feed_topic = self.get_parameter('perception.camera_interface.camera_feed').get_parameter_value().string_value
        self.img_pub = self.create_publisher(Image, camera_feed_topic, 1)

    def declareParameters(self) -> None:
        """
        Declare parameters for camera interface node
        """
        self.declare_parameter("perception.camera_interface.frame_id", rclpy.Parameter.Type.STRING)
        self.declare_parameter("perception.camera_interface.camera_feed", rclpy.Parameter.Type.STRING)
        # Video feed params
        self.declare_parameter("camera.image_width", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("camera.image_height", rclpy.Parameter.Type.INTEGER)
        # Camera Intrinsics
        self.declare_parameter("camera.fx", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.fy", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.cx", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.cy", rclpy.Parameter.Type.DOUBLE)
        # Distortion coefficients
        self.declare_parameter("camera.k1", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.k2", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.p1", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("camera.p2", rclpy.Parameter.Type.DOUBLE)

    def setParameters(self) -> None:
        """
        Fetches camera interface parameters, initializes CvBridge, cv2 cap and generates undistortion maps
        """
        
        self.frameId = self.get_parameter("perception.camera_interface.frame_id").get_parameter_value().string_value
        self.bridge = CvBridge()

        # Video feed params
        width = self.get_parameter('camera.image_width').get_parameter_value().integer_value
        height = self.get_parameter('camera.image_height').get_parameter_value().integer_value
        # Camera intrinsics
        cx = self.get_parameter('camera.cx').get_parameter_value().double_value
        cy = self.get_parameter('camera.cy').get_parameter_value().double_value
        fx = self.get_parameter('camera.fx').get_parameter_value().double_value
        fy = self.get_parameter('camera.fy').get_parameter_value().double_value
        # Distortion coefficients
        k1 = self.get_parameter('camera.k1').get_parameter_value().double_value
        k2 = self.get_parameter('camera.k2').get_parameter_value().double_value
        p1 = self.get_parameter('camera.p1').get_parameter_value().double_value
        p2 = self.get_parameter('camera.p2').get_parameter_value().double_value


        # Generate Undistortion Maps
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]) 
        dist_coeffs = np.array([[k1], [k2], [p1], [p2]])
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, (width, height), cv2.CV_32FC1)

        #set-up camera capture 
        self.cap = cv2.VideoCapture(-1)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            sys.exit(1)

        #set camera capture resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        #set camera capture FPS
        self.cap.set(cv2.CAP_PROP_FPS, 10)

    def run(self) -> None:
        """
        runs the camera interface node, retrieve left image from stereo pair and undistorts image
        """
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Error capturing image')
                return
            # extract the left image from the ZED's stereo pair
            left_img = np.split(frame, 2, axis=1)[0]

            #undistort
            undistorted_img = cv2.remap(left_img, self.mapx, self.mapy, cv2.INTER_LINEAR)
            rgb_img = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
            
            # Convert OpenCV image to ROS2 Image message
            ros_image = self.bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
            ros_image.header.frame_id = str(self.ViewId)
            
            # Publish the image
            self.img_pub.publish(ros_image)

            self.get_logger().info('Publishing camera feed frame: ' + str(self.ViewId))
            self.ViewId += 1
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rclpy.init()
    camera_node = CameraNode()
    status = StatusPublisher("/status/camera_interface", camera_node)

    thread = threading.Thread(target=rclpy.spin, args=(camera_node, ), daemon=True)
    thread.start()

    status.starting()

    # Publish heartbeat to show the module is ready
    status.ready()

    # Main loop
    rate = camera_node.create_rate(frequency=10)
    while rclpy.ok():
        rate.sleep()
        camera_node.run()
        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    main()
