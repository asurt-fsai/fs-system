import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_processor')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/camera_interface/camera_feed', rclpy.Parameter.Type.STRING),
                ('/video_processor/video_path', rclpy.Parameter.Type.STRING),
                ('/video_processor/image_width', rclpy.Parameter.Type.INTEGER),
                ('/video_processor/image_height', rclpy.Parameter.Type.INTEGER),
                ('/video_processor/callback_timer', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        
        self.frame_id = None
        self.w = None
        self.h = None
        self.bridge = None
        self.cap = None
        self.img_publisher_ = None
        self.timer = None
        
        
        self.start()
        
        
        
        
    def start(self):
        
        publisher_topic = self.get_parameter('/camera_interface/camera_feed').get_parameter_value().string_value
        video_path = self.get_parameter('/video_processor/video_path').get_parameter_value().string_value
        self.w = self.get_parameter('/video_processor/image_width').get_parameter_value().integer_value
        self.h = self.get_parameter('/video_processor/image_height').get_parameter_value().integer_value
        timer_period = self.get_parameter('/video_processor/callback_timer').get_parameter_value().double_value
        
        self.frame_id=0
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_path)  # Specify the path to your video file
        
        self.img_publisher_ = self.create_publisher(Image, publisher_topic, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame,(self.w, self.h))
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