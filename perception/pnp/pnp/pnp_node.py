import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from asurt_msgs.msg import Landmark, LandmarkArray, KeyPoints
from nav_msgs.msg import Odometry
from transformations import quaternion_matrix

class PnpNode(Node):

    def __init__(self):
        
        super().__init__("pnp_node")
        self.get_logger().info("STARTING PNP NODE")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/camera_interface/cx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/cy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fx', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/fy', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p1', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/p2', rclpy.Parameter.Type.DOUBLE),
                ('/camera_interface/k3', rclpy.Parameter.Type.DOUBLE),
                #cone dimensions
                ('/slam/odom', rclpy.Parameter.Type.STRING),
                ('/kpr/keypoints_topic', rclpy.Parameter.Type.STRING),
                ('/pnp/landmark_array', rclpy.Parameter.Type.STRING)
            ]
        )

        self.odom_subscriber = None
        self.odom_msg = None
        self.Tr = None #a static transform matrix from velodyne to ZED left camera
        
        self.kpr_subscriber = None
        self.kpr_msg = None
        self.classes = None
        self.track_ids = None 

        self.landmark_array_publisher = None


        self.camera_matrix = None
        self.dist_coeffs = None
        self.big_cone_dims = None 
        self.small_cone_dims = None

        self.start()

    def get_camera_params(self):
        try:
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
            camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]) 
            dist_coeffs = np.array([k1], [k2], [p1], [p2], [k3])

            # Printing fetched parameters for confirmation
            self.get_logger().info(f"Fetched camera parameters successfully: {camera_matrix}")
            return camera_matrix, dist_coeffs

        except Exception as e:
            # Logging the error and re-raising
            self.get_logger().error(f"Failed to fetch parameters: {e}")

    def start(self):
        
        # fetch parameters from launch file
        odom_topic = self.get_parameter("/slam/odom").get_parameter_value().string_value
        kpr_topic = self.get_parameter("/kpr/keypoints_topic").get_parameter_value().string_value
        landmark_array_topic = self.get_parameter("/moreo/landmark_array").get_parameter_value().string_value
        self.camera_matrix, self.dist_coeffs = self.get_camera_params()

        # fetch cone params
        big_cone_dims = eval(self.get_parameter("/pnp/big_cone_dims").get_parameter_value().string_value)
        self.big_cone_dims = np.array(big_cone_dims)

        small_cone_dims =eval(self.get_parameter("/pnp/small_cone_dims").get_parameter_value().string_value) 
        self.small_cone_dims = np.array(small_cone_dims)

        # fetch Tr

        # define subscribers & publisher
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.callback_odom, 10)
        self.kpr_subscriber = self.create_subscription(KeyPoints, kpr_topic, self.callback_kpr, 10)
        self.landmark_array_publisher = self.create_publisher(LandmarkArray, landmark_array_topic, 10)


    def callback_odom(self, msg:Odometry):

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
        
        self.odom_msg = transformation_matrix_camera_frame[0:3, 3] # return xyz position only
        self.callback_pnp()


    def callback_kpr(self, msg:KeyPoints):
        
        object_count = msg.object_count
        self.kpr_msg = np.array(msg.keypoints).reshape(object_count, 7, 2)
        self.classes = msg.classes
        self.callback_moreo()

    def callback_pnp(self):

        if self.odom_msg and self.kpr_msg:
            self.get_logger().info(f'Ready to process: cone keypoints & odometry data')
            
            landmarks = LandmarkArray()

            for i, kps in enumerate(self.kpr_msg):
                
                object_points = self.small_cone_dims

                if (self.classes[i] == 2):
                    object_points = self.big_cone_dims

                _, _, tvec, _ = cv2.solvePnPRansac(
                    object_points, 
                    kps, 
                    self.camera_matrix, 
                    self.dist_coeffs.reshape(5), 
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                landmark = Landmark()
                landmark.identifier = self.track_ids[i]
                landmark.position = tvec + self.odom_msg
                landmarks.landmarks.append(landmark)

            # publish landmark array
            self.landmark_array_publisher.publish(landmarks) 
            # Reset messages 
            self.odom_msg = None
            self.kpr_msg = None

def main(args=None):

    rclpy.init(args=args)
    node = PnpNode()
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