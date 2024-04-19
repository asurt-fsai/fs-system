#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from simple_pure_pursuitController import SimplePurePursuit
from tf_helper.TFHelper import TFHelper


class SimplePurePursuitNode(Node):  # type: ignore[misc]
    """
        Class to run the simple pure pursuit controller node
         Main function for simple pure pursuit vehicle control node, subscribes to
    #     waypoints and publishes control actions
    """

    def __init__(self) -> None:
        """
        Initilization of the simple pure pursuit node
        """
        super().__init__("simple_pure_pursuit_node")
        self.controller = SimplePurePursuit(self)

        self.targetSpeed = self.get_parameter("/control/speed_target")
        # self.controlRate = rclpy.Parameter.get_parameter_value('/control/rate')
        controlTopic = self.get_parameter("/control/actions_topic")

        self.rate = self.create_rate(1)  # self.controlRate)
        self.tfHelper = TFHelper("control")
        self.markerPub = self.create_publisher(Marker, "/marker_viz", 10)
        self.controlActionPub = self.create_publisher(AckermannDriveStamped, controlTopic, 10)
        self.steeringPub = self.create_publisher(Float32, "/steer", 10)
        self.create_subscription(Path, "/waypoints", self.controller.add, 10)

    def pathCallback(self, path: Path) -> None:
        """
        Call back function for path msg
        """
        path = self.tfHelper.transformMsg(path, "base_link")
        if len(path.poses) > 0:
            self.controller.add(path)

            delta, ind = self.controller.purepursuitSteercontrol()
            controlMsg = AckermannDriveStamped()
            controlMsg.drive.speed = self.targetSpeed
            controlMsg.drive.steer = delta
            controlMsg.header.stamp = self.get_clock().now()
            self.visualizer(ind)
            self.controlActionPub.publish(controlMsg)
            self.get_logger().info("Publishing")

    def visualizer(self, ind: int) -> None:
        """
        Visualize the targeted waypoint along the path

        parameters:
        ind (int) : target index
        """

        if len(self.controller.xList) > 0:

            vizPoint = Marker()
            vizPoint.header.frame_id = "base_link"
            vizPoint.header.stamp = self.get_clock().now()
            vizPoint.ns = "pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = self.controller.xList[ind]
            vizPoint.pose.position.y = self.controller.yList[ind]
            vizPoint.pose.position.z = 0.0
            vizPoint.pose.orientation.x = 0.0
            vizPoint.pose.orientation.y = 0.0
            vizPoint.pose.orientation.z = 0.0
            vizPoint.pose.orientation.w = 1.0
            vizPoint.scale.x = 0.5
            vizPoint.scale.y = 0.5
            vizPoint.scale.z = 0.5
            vizPoint.color.r = 1.0
            vizPoint.color.a = 1.0
            self.markerPub.publish(vizPoint)


def main(args=None) -> None:  # type: ignore[no-untyped-def]
    """
    Main function for simple pure pursuit vehicle control node
    """
    rclpy.init(args=args)

    simplePPNode = SimplePurePursuitNode()

    while rclpy.ok():

        rclpy.spin(simplePPNode)

    simplePPNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
