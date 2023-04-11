#!/usr/bin/python3
"""
.

"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped


class Republisher:
    """
    This claas republishes the steer and vel to the car
    after it has been received from the commander
    """

    def __init__(self) -> None:
        self.cmd = rospy.Publisher("/cmd", AckermannDriveStamped, queue_size=10)
        self.vel = 0
        self.steer = 0

    def publishCmd(self) -> None:
        """
        Publishes the command to the car
        after changing the steer and vel
        msg to AckermannDriveStamped
        """
        cmdMsg = AckermannDriveStamped()
        cmdMsg.drive.speed = self.vel
        cmdMsg.drive.steering_angle = self.steer
        self.cmd.publish(cmdMsg)

    def velCallback(self, msg: AckermannDriveStamped) -> None:
        """
        Callback function for the velocity
        """
        self.vel = msg.data
        self.publishCmd()

    def steerCallback(self, msg: AckermannDriveStamped) -> None:
        """
        Callback function for the steering angle
        """
        self.steer = msg.data
        self.publishCmd()
