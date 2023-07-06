#!/usr/bin/python3
"""
Main ROS node for publishing skidpad waypoints
"""

from typing import Any
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from skidpad.skidpad import Skidpad


class WaypointsNode:
    """
    ROS node for publishing skidpad waypoints.
    """

    def __init__(self) -> None:
        self.carPosition = [0, 0]
        self.waypoints: Any = []
        self.index = 0

    def updateCarPosition(self, data: PoseStamped) -> None:
        """
        Update the position of the car based on the received data

        Parameters:
        ------------
        data : PoseStamped
            Data containing the car's position received from the "/car_position" topic.

        Returns:
        -----------
        None
        """
        self.carPosition[0] = data.pose.position.x
        self.carPosition[1] = data.pose.position.y
        distanceX = (self.carPosition[0] - self.waypoints[self.index][0]) ** 2
        distanceY = (self.carPosition[1] - self.waypoints[self.index][1]) ** 2
        if np.sqrt(distanceX + distanceY) < 0.1:
            if self.index + 11 < len(self.waypoints):
                self.index += 1

    def run(self) -> None:
        """
        Run the waypoints node.

        Parameters:
        ------------
        None

        Returns:
        -----------
        None
        """
        rospy.init_node("waypoints_node")
        publisher = rospy.Publisher("/waypoints_topic", Path, queue_size=10)
        rospy.Subscriber("/car_position", PoseStamped, self.updateCarPosition)
        rate = rospy.Rate(10)
        distance = rospy.get_param("/waypoints_node/distance")
        innerRadius = rospy.get_param("/waypoints_node/inner_radius")
        outerRadius = rospy.get_param("/waypoints_node/outer_radius")
        lengthOfLineT = rospy.get_param("/waypoints_node/len_of_line_t")
        lengthOfLineB = rospy.get_param("/waypoints_node/len_of_line_b")
        step = rospy.get_param("/waypoints_node/step")

        skidpad = Skidpad(distance, innerRadius, outerRadius, lengthOfLineB, lengthOfLineT)
        self.waypoints = skidpad.getWaypoints(step)

        while not rospy.is_shutdown():
            pathMsg = Path()
            points = []
            if self.index + 10 < len(self.waypoints):
                for point in self.waypoints[self.index : self.index + 10]:
                    pointMsg = PoseStamped()
                    pointMsg.pose.position.x = point[0]
                    pointMsg.pose.position.y = point[1]
                    pointMsg.header.frame_id = "map"
                    pointMsg.header.stamp = rospy.Time.now()
                    points.append(pointMsg)
                pathMsg.poses = points
                pathMsg.header.frame_id = "map"
                pathMsg.header.stamp = rospy.Time.now()
                publisher.publish(pathMsg)
            rate.sleep()


if __name__ == "__main__":
    node = WaypointsNode()
    node.run()
