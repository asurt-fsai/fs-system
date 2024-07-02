# pylint: skip-file
# type: ignore
"""generates path and publishes it to /path topic and
generates track width with different color than path"""

import math
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathGenerator(Node):
    """Generates path and publishes it to /path topic.
    cYcomponent = [math.sin(ix / 5.0) * ix / 2.0 for ix in cXcomponent]
    self.path.header.frame_id = "map"ishes it to /path topic."""

    def __init__(self) -> None:
        super().__init__("PathGenerator")
        self.get_logger().info("Path Generator Node Initialized")
        self.pathPub = self.create_publisher(Path, "/path", 10)
        self.path = Path()
        self.publishGen()
        self.timer = self.create_timer(0.1, self.publishPath)

    def publishGen(self) -> None:
        """Generates path.
        args:
            None
        returns:
            None
        """
        cXcomponent = np.arange(0, 80, 0.5)
        cYcomponent = [math.sin(ix / 5.0) * ix / 2.0 for ix in cXcomponent]
        self.path.header.frame_id = "map"
        self.path.header.stamp = self.get_clock().now().to_msg()
        for i in enumerate(cXcomponent):
            pose = PoseStamped()
            pose.pose.position.x = cXcomponent[i]
            pose.pose.position.y = cYcomponent[i]
            self.path.poses.append(pose)

    def publishPath(self) -> None:
        """Publishes path to /path topic."""
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pathPub.publish(self.path)


def main(args=None) -> None:
    """
    Main function to initialize the node and the path generator object.
    args:
        args: list
    returns:
        None
    """
    rclpy.init(args=args)
    pathGen = PathGenerator()
    rclpy.spin(pathGen)
    pathGen.destroy_node()
    rclpy.shutdown()
