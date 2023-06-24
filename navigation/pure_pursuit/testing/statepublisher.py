#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf_helper import TFHelper
import tf

BaseLength = 2.9
dt = 0.1


class State:
    """

    state of the vehicle gotten from SLAM

    """

    def __init__(
        self, x: float = -2.0, y: float = 0.0, yaw: float = 0.0, currentSpeed: float = 0.0
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.x - ((BaseLength / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseLength / 2) * math.sin(self.yaw))

    def update(self, controlActions: AckermannDriveStamped) -> None:
        """

        update the state of the vehicle

        """

        self.currentSpeed += controlActions.drive.acceleration * dt
        self.x += self.currentSpeed * math.cos(self.yaw) * dt
        self.y += self.currentSpeed * math.sin(self.yaw) * dt
        self.yaw += (
            self.currentSpeed / BaseLength * math.tan(controlActions.drive.steering_angle) * dt
        )

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


def handle__pose(msg: PoseStamped, frameename):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (msg.pose.position.x, msg.pose.position.y, 0),
        tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z),
        rospy.Time.now(),
        frameename,
        "map",
    )


if __name__ == "__main__":
    state = State()
    rospy.init_node("statepublisher", anonymous=True)

    rospy.Subscriber("/control_actions", AckermannDriveStamped, state.update)
    pub = rospy.Publisher("/state", Odometry, queue_size=10)
    rate = rospy.Rate(10)
    message = Odometry()

    while not rospy.is_shutdown():
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.orientation.x = state.currentSpeed
        message.pose.pose.orientation.z = state.yaw
        message.header.frame_id = "map"
        pub.publish(message)
        handle__pose(message, "base_link")
        rate.sleep()
