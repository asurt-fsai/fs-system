#!/usr/bin/python
"""
This script initializes the acceleration node and publishes the acceleration topic
"""
from typing import Tuple
import rospy
from skidpad_acceleration.acceleration import Acceleration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np


class SkidpadAcceleration:
    """
    This class initializes the acceleration node and publishes the acceleration topic
    """

    def __init__(self) -> None:
        self.carPosition: Tuple[int, int] = (0, 0)
        self.wayPoint: Path = []
        self.index: int = 0

    def updateCarPosition(self, data: PoseStamped) -> None:
        """
        This method updates the car's position
            Parameters:
            ----------
            data: PoseStamped
        """
        self.carPosition = (data.pose.position.x, data.pose.position.y)
        if self.index + 11 < len(self.wayPoint):
            if (
                np.sqrt(
                    (self.carPosition[0] - self.wayPoint[self.index][0]) * 2
                    + (self.carPosition[1] - self.wayPoint[self.index][1]) * 2
                )
                < 0.1
            ):
                self.index += 1

    def main(self) -> None:
        """
        This method initializes the acceleration node and publishes the acceleration topic
        """
        lenOfLine = 100
        acceleration = Acceleration(lenOfLine)
        self.wayPoint = acceleration.getWaypoints()
        rospy.init_node("acceleration_node")
        publisher = rospy.Publisher("acceleration_topic", Path, queue_size=10)
        rospy.Subscriber("/car_position", PoseStamped, self.updateCarPosition)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pathMsg = Path()
            points = []
            if self.index + 10 < len(self.wayPoint):
                for point in self.wayPoint[self.index : self.index + 10]:
                    poseMsg = PoseStamped()
                    poseMsg.pose.position.x = point[0]
                    poseMsg.pose.position.y = point[1]
                    poseMsg.header.frame_id = "map"
                    poseMsg.header.stamp = rospy.Time.now()
                    points.append(poseMsg)
                pathMsg.poses = points
                pathMsg.header.frame_id = "map"
                pathMsg.header.stamp = rospy.Time.now()
                publisher.publish(pathMsg)
            rate.sleep()


if __name__ == "__main__":
    try:
        skidpadAcc = SkidpadAcceleration()
        skidpadAcc.main()
    except rospy.ROSInterruptException:
        pass
