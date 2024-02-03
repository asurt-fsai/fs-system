import do_mpc
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive


class mpc_cd1(Node):
    def _init_(self):
        super().__init__('mpc_cd1')

        #self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        #self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.control_pub = self.create_publisher(AckermannDrive , "/control", 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.timer = self.create_timer(0.1,self.publish_control_signals)
        
        
    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        self.velocity = math.sqrt(Vx * 2 + Vy * 2)
        X = state.pose.pose.position.x
        Y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.state = (X, Y, yaw)
        throttle_msg = Float32()
        throttle_msg.data, steer_msg = self.mpc_control()
        self.throttle_pub.publish(throttle_msg)

    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True
