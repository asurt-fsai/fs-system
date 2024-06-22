# pylint: disable=all
# mypy: ignore-errors
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node


p1 = PoseStamped()
p1.pose.position.x = 1.0
p1.pose.position.y = 1.0
p1.header.frame_id = "map"
p2 = PoseStamped()
p2.pose.position.x = 2.0
p2.pose.position.y = 2.0
p2.header.frame_id = "map"
p3 = PoseStamped()
p3.pose.position.x = 3.0
p3.pose.position.y = 3.0
p3.header.frame_id = "map"
p4 = PoseStamped()
p4.pose.position.x = 4.0
p4.pose.position.y = 4.0
p4.header.frame_id = "map"
p5 = PoseStamped()
p5.pose.position.x = 5.0
p5.pose.position.y = 5.0
p5.header.frame_id = "map"
p6 = PoseStamped()
p6.pose.position.x = 6.0
p6.pose.position.y = 6.0
p6.header.frame_id = "map"
p7 = PoseStamped()
p7.pose.position.x = 7.0
p7.pose.position.y = 7.0
p7.header.frame_id = "map"
p8 = PoseStamped()
p8.pose.position.x = 8.0
p8.pose.position.y = 8.0
p8.header.frame_id = "map"
p9 = PoseStamped()
p9.pose.position.x = 9.0
p9.pose.position.y = 9.0
p9.header.frame_id = "map"
p10 = PoseStamped()
p10.pose.position.x = 10.0
p10.pose.position.y = 10.0
p10.header.frame_id = "map"
p11 = PoseStamped()
p11.pose.position.x = 11.0
p11.pose.position.y = 11.0
p11.header.frame_id = "map"

poses = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]


class WayPoints(Node):
    def __init__(self):
        super().__init__("waypoints_node")
        self.waypoints = Path()
        self.msgPublisher = self.create_publisher(Path, "/waypoints", 10)
        self.create_timer(1, self.publishing)
        self.waypoints.poses = poses
        self.waypoints.header.frame_id = "map"
        # self.waypoints.header.stamp = rclpy.Node.get_clock().now()

    def publishing(self):
        # self.get_logger().info("Publishing Paths")

        self.msgPublisher.publish(self.waypoints)


def main(args=None):
    rclpy.init(args=args)

    waypoints = WayPoints()
    # while rclpy.ok():
    #     # simple_pure_pursuit_node.get_logger().info("Publishing")
    #     simple_pure_pursuit_node.loop()
    #     waypoints_node.publishing()
    #     # waypoints_node.get_logger().info("Publishing Waypoints")

    # rclpy.spin(waypoints_node)
    rclpy.spin(waypoints)
    # waypoints_node.destroy_node()
    waypoints.destroy_node()
    rclpy.shutdown()
