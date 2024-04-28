import rclpy
import numpy as np

from rclpy.node import Node
from asurt_msgs.msg import Landmark, LandmarkArray, KeyPoints
from nav_msgs.msg import Odometry
from transformations import quaternion_matrix
from moreo_logic import *

class MoreoNode(Node):

    def __init__(self):
        
        super().__init__("moreo_node")
        self.get_logger().info("STARTING MOREO NODE")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/slam/odom', rclpy.Parameter.Type.STRING),
                ('/kpr/keypoints_topic', rclpy.Parameter.Type.STRING),
                ('/moreo/landmark_array', rclpy.Parameter.Type.STRING),
                ('/moreo/min_baseline', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        self.odom_subscriber = None
        self.kpr_subscriber = None
        self.landmark_array_publisher = None

        self.odom_msg = None
        self.kpr_msg = None

        self.Tr = None #a static transform matrix from velodyne to ZED left camera

        self.start()

    def start(self):
        
        # fetch parameters from launch file
        odom_topic = self.get_parameter("/slam/odom").get_parameter_value().string_value
        kpr_topic = self.get_parameter("/kpr/keypoints_topic").get_parameter_value().string_value
        landmark_array_topic = self.get_parameter("/moreo/landmark_array").get_parameter_value().string_value
        min_baseline = self.get_parameter("/moreo/min_baseline").get_parameter_value().double_value

        # define subscribers & publisher
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.callback_odom, 10)
        self.kpr_subscriber = self.create_subscription(KeyPoints, kpr_topic, self.callback_kpr, 10)
        self.landmark_array_publisher = self.create_publisher(LandmarkArray, landmark_array_topic, 10)

        #fetch Tr somehow

    def callback_odom(self, msg: Odometry):

        # Extract the pose part from the Odometry message
        pose = msg.pose.pose

        # Extract position data
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        
        # Extract orientation data (quaternion)
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        # Convert quaternion to rotation matrix
        rotation_matrix = quaternion_matrix([qx, qy, qz, qw])

        # Create homogeneous transformation matrix
        transformation_matrix = np.eye(4)  # Start with an identity matrix
        transformation_matrix[0:3, 0:3] = rotation_matrix[0:3, 0:3]  # Set rotation part
        transformation_matrix[0:3, 3] = [px, py, pz]  # Set translation part

        #transform odometry information from velodyne to ZED left camera coordinate frame
        transformation_matrix_camera_frame = np.dot(self.Tr, transformation_matrix)
        
        self.odom_msg = transformation_matrix_camera_frame
        self.callback_moreo()

    def callback_kpr(self, msg: KeyPoints):
        
        kpr_arr = msg
        object_count = kpr_arr.object_count
        kpr_data = np.array(kpr_arr.keypoints)
        self.kpr_msg = kpr_data.reshape(object_count, 7, 2)
        self.callback_moreo()

    def callback_moreo(self):
        if self.odom_msg and self.kpr_msg:
            self.get_logger().info(f'Ready to process: cone keypoints & odometry data')
            
            # Reset messages 
            self.odom_msg = None
            self.kpr_msg = None


def main(args=None):

    rclpy.init(args=args)
    node = MoreoNode()
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