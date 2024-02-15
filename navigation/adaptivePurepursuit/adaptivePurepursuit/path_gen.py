####generates path and publishes it to /path topic

### generates track width with different color than path

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import rclpy
from rclpy.node import Node


class PathGenerator(Node):
    def __init__(self):
        super().__init__('PathGenerator')
        self.get_logger().info('Path Generator Node Initialized')
        self.pathPub = self.create_publisher(
            Path, '/path', 10)
        self.path = Path()
        self.publishGen()
        self.timer = self.create_timer(0.1, self.publishPath)
        

    def publishGen(self):
        cx = np.arange(0, 80, 0.5)
        cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
        self.path.header.frame_id = "map"

        self.path.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(cx)):
            pose = PoseStamped()
            pose.pose.position.x = cx[i]
            pose.pose.position.y = cy[i]
            self.path.poses.append(pose)
        
    def publishPath(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pathPub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    pathGen = PathGenerator()
    rclpy.spin(pathGen)
    pathGen.destroy_node()
    rclpy.shutdown()
