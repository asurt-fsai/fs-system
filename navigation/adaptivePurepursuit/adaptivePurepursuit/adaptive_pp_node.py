import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
import math
from tf_transformations import euler_from_quaternion
from adaptivePurepursuit import AdaptivePurePursuit

from ackermann_msgs.msg import AckermannDrive

# node file , purepursuit file
# launch file with parameters
# search index function
# pkg folder structure

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        
        self.controlActionsPub = self.create_publisher(AckermannDrive, 'control/actions', 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_control_signals)

        #state_callback function initialization

        self.purepursuit = AdaptivePurePursuit()
        #pid_controller function initialization


        #adaptivePurepursuit function initialization

    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        self.purepursuit.velocity = math.sqrt(Vx ** 2 + Vy ** 2)
        self.purepursuit.x = state.pose.pose.position.x
        self.purepursuit.y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, self.purepursuit.yaw = euler_from_quaternion(orientation_list)
        

    def path_callback(self, path: Path):
        self.purepursuit.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.purepursuit.pathFlag = True

    def publish_control_signals(self):
        steering_angle = Float32()
        steering_angle.data = self.purepursuit.adaptivePurepursuit()

        throttle = Float32()
        throttle.data = self.purepursuit.pid_controller(steering_angle.data)
        self.throttle_pub.publish(throttle)
        self.steer_pub.publish(steering_angle)
        # if self.pathFlag == True :
        #     steering_angle = Float32()
        #     steering_angle.data = self.adaptivePurepursuit()*(180/math.pi)
        #     self.steer_pub.publish(steering_angle)
        #     if len(self.waypoints) > 0 and self.search_target_point == len(self.waypoints) - 1:
        #         throttle = 0
        #     else:
        #         throttle = Float32()
        #         throttle.data = self.pid_controller(steering_angle.data)
        #     self.throttle_pub.publish(throttle)
        #     log = "tracking waypoint: " + str(self.waypoints[self.i])
        #     self.get_logger().info(log)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()