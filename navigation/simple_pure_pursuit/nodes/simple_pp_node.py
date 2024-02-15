#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from simple_pure_pursuit import SimplePurePursuit  # type: ignore[attr-defined]

import rclpy
from rclpy.node import Node

class SimplePurePursuitNode(Node):

    def __init__(self):
        super().__init__('simple_pure_pursuit_node')
        self.controller = SimplePurePursuit()
        self.markerPub = self.create_publisher(Marker, '/marker_viz', 10)
        self.controlActionPub = self.create_publisher(AckermannDriveStamped, '/control/actions', 10)
        self.steeringPub = self.create_publisher(Float32, '/steer', 10)
        self.create_subscription(Path, '/waypoints', self.callback, 10)
        self.targetSpeed = rclpy.Parameter.get_parameter_value('/control/speed_target')
        self.controlRate = rclpy.Parameter.get_parameter_value('/control/rate')
        self.rate = self.create_rate(self.controlRate)
def main() -> None:
    """
    Main function for simple pure pursuit vehicle control node, subscribes to
    waypoints and publishes control actions
    """
    rospy.init_node("simple_pp_controller", anonymous=True)

    controlTopic = rospy.get_param("/control/actions_topic")
    waypointsTopic = rospy.get_param("/control/waypoints_topic")
    markerVizTopic = rospy.get_param("/control/marker_viz_topic")

    targetSpeed = rospy.get_param("/control/speed_target")
    controlRate = rospy.get_param("/control/rate")

    controller = SimplePurePursuit()
    markerPub = rospy.Publisher(markerVizTopic, Marker, queue_size=10)
    controlActionPub = rospy.Publisher(controlTopic, AckermannDriveStamped, queue_size=10)
    rospy.Subscriber(waypointsTopic, Path, callback=controller.add)
    steeringPub = rospy.Publisher("/steer", Float32, queue_size=10)
    rate = rospy.Rate(controlRate)

    while not rospy.is_shutdown():
        delta, ind = controller.purepursuitSteercontrol()

        controlAction = AckermannDriveStamped()
        controlAction.header.stamp = rospy.Time.now()
        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = targetSpeed
        controlActionPub.publish(controlAction)
        steerMsg = Float32()
        steerMsg.data = delta
        steeringPub.publish(steerMsg)
        if len(controller.xList) > 0:
            vizPoint = Marker()
            vizPoint.header.frame_id = "rear_link"
            vizPoint.header.stamp = rospy.Time.now()
            vizPoint.ns = "pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = controller.xList[ind]
            vizPoint.pose.position.y = controller.yList[ind]
            vizPoint.pose.position.z = 0
            vizPoint.pose.orientation.x = 0.0
            vizPoint.pose.orientation.y = 0.0
            vizPoint.pose.orientation.z = 0.0
            vizPoint.pose.orientation.w = 1.0
            vizPoint.scale.x = 0.5
            vizPoint.scale.y = 0.5
            vizPoint.scale.z = 0.5
            vizPoint.color.r = 1.0
            vizPoint.color.a = 1.0
            markerPub.publish(vizPoint)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
