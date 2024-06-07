import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_processor')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/video_processor/video_path', rclpy.Parameter.Type.STRING),
                ('/video_processor/image_width', rclpy.Parameter.Type.INTEGER),
                ('/video_processor/image_height', rclpy.Parameter.Type.INTEGER),
                ('/video_processor/callback_timer', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/cx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/cy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k3', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/camera_feed', rclpy.Parameter.Type.STRING),
                
            ]
        )
        
        self.frame_id = None
        self.params = None
        self.bridge = None
        self.cap = None
        self.img_publisher_ = None
        self.timer = None
        
        self.mapx = None
        self.mapy = None
        
        
        self.start()
         
        
    def get_params(self):
        try:
            # Video feed params
            video_path = self.get_parameter('/video_processor/video_path').get_parameter_value().string_value
            width = self.get_parameter('/video_processor/image_width').get_parameter_value().integer_value
            height = self.get_parameter('/video_processor/image_height').get_parameter_value().integer_value
            rate = self.get_parameter('/video_processor/callback_timer').get_parameter_value().double_value
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
                'video_path':video_path,
                'width': width,
                'height': height,
                'rate':rate,
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
            
    
    def generate_undistort_maps(self):

        camera_matrix = np.array([[self.params['fx'], 0, self.params['cx']], [0, self.params['fy'], self.params['cy']], [0, 0, 1]]) 
        dist_coeffs = np.array([[self.params['k1']], [self.params['k2']], [self.params['p1']], [self.params['p2']], [self.params['k3']]])

        # Generate undistortion maps
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, (self.params['width'], self.params['height']), cv2.CV_32FC1)
        
                 
    def start(self):
        
        publisher_topic = self.get_parameter('/camera_interface/camera_feed').get_parameter_value().string_value
        
        self.params = self.get_params()
        
        
        self.frame_id=0
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.params['video_path'])  # Specify the path to your video file
        
        self.img_publisher_ = self.create_publisher(Image, publisher_topic, 10)
        self.timer = self.create_timer(self.params['rate'], self.timer_callback)
        
        self.generate_undistort_maps()
        
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame,(self.params['width'], self.params['height']))
            frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            ros_image.header.frame_id = str(self.frame_id)
            self.img_publisher_.publish(ros_image)
            self.get_logger().info(f'Published frame: {self.frame_id}')
            self.frame_id += 1
        else:
            self.get_logger().info('No frame received. Exiting...')
            self.cap.release()
            rclpy.shutdown()
            
def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()