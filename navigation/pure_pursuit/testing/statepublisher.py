#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDrive

BaseLength = 2.9
dt = 0.1


class State:
    """

    state of the vehicle gotten from SLAM

    """

    def __init__(
        self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, currentSpeed: float = 0.0
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.x - ((BaseLength / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseLength / 2) * math.sin(self.yaw))

    def update(self, controlActions: AckermannDrive) -> None:
        """

        update the state of the vehicle

        """

        self.currentSpeed += controlActions.acceleration * dt
        self.x += self.currentSpeed * math.cos(self.yaw) * dt
        self.y += self.currentSpeed * math.sin(self.yaw) * dt
        self.yaw += self.currentSpeed / BaseLength * math.tan(controlActions.steering_angle) * dt

        self.rearX = self.x - ((BaseLength / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseLength / 2) * math.sin(self.yaw))
        return None

    def calcDistance(self, point_x: float, point_y: float) -> float:
        """

        calculate the distance between the vehicle and the target point

        """
        dx = self.rearX - point_x
        dy = self.rearY - point_y
        return math.hypot(dx, dy)


if __name__ == "__main__":
    state = State()
    rospy.init_node("statepublisher", anonymous=True)
    rospy.Subscriber("/control_actions", AckermannDrive, state.update)
    pub = rospy.Publisher("/state", Pose, queue_size=10)
    rate = rospy.Rate(10)
    message = Pose()
    while not rospy.is_shutdown():
        message.position.x = state.x
        message.position.y = state.y
        message.orientation.x = state.currentSpeed
        message.orientation.z = state.yaw
        pub.publish(message)
        rate.sleep()
