#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
# from typing import Tuple
from nav_msgs.msg import Path

# from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf_helper.StatusPublisher import StatusPublisher

# from std_msgs.msg import Float32
from simple_purepursuitcontroller import SimplePurePursuit
from ackermann_msgs.msg import AckermannDrive
import rclpy
from rclpy.node import Node

# mypy: disable-error-code="misc"
class SimplePurePursuitNode(Node):
    """node for simple pure pursuit"""

    def __init__(self) -> None:
        super().__init__("simple_pure_pursuit_node")
        self.controller = SimplePurePursuit(self)

        self.declare_parameter("control.cmd", rclpy.Parameter.Type.STRING)

        markervizTopic = self.get_parameter("control.marker_viz").get_parameter_value().string_value
        actionsTopic = self.get_parameter("control.cmd").get_parameter_value().string_value
        waypointsTopic = (
            self.get_parameter("planning.waypoints_clean").get_parameter_value().string_value
        )  # no2ta baadal /

        self.markerPub = self.create_publisher(Marker, markervizTopic, 10)
        # eltopic n5aleh variable bardo
        self.steeringPub = self.create_publisher(AckermannDrive, actionsTopic, 10)
        self.create_subscription(Path, waypointsTopic, self.controller.add, 10)  # waypoints
        self.create_timer(0.1, self.action)

    def action(self) -> None:  # pylint: disable=[E1126]
        """publishing steering angle"""

        delta, ind = self.controller.purepursuitSteercontrol()
        steerMsg = AckermannDrive()
        steerMsg.steering_angle = delta
        steerMsg.speed = 1.0
        self.steeringPub.publish(steerMsg)

        if len(self.controller.xList) > 0:
            vizPoint = Marker()
            vizPoint.header.frameId = self.controller.frameId

            vizPoint.ns = "pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = self.controller.xList[ind]  # pylint: disable=[E1126]
            vizPoint.pose.position.y = self.controller.yList[ind]  # pylint: disable=[E1126]
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


def main() -> None:
    """main function to intiallize node and controll the car"""
    rclpy.init(args=None)
    simplepurePursuitnode: SimplePurePursuitNode = SimplePurePursuitNode()
    status: StatusPublisher = StatusPublisher("/status/simple_pure_pursuit", simplepurePursuitnode)
    status.starting()
    status.ready()
    while rclpy.ok():
        rclpy.spin(simplepurePursuitnode)

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    main()
