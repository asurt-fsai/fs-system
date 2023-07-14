#!/usr/bin/python3
"""
Static A
"""
import time

import rospy
import numpy as np

from std_msgs.msg import Float32, Bool, Int16


class StaticA:
    """
    Static A
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(
        self, velTopic: str, steerTopic: str, isFinishedTopic: str, stateTopic: str
    ) -> None:
        rospy.loginfo("Starting Static A")

        self.drivingFlag = False
        self.started = False

        self.velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        self.steerPub = rospy.Publisher(steerTopic, Float32, queue_size=10)
        self.statePub = rospy.Publisher(stateTopic, Int16, queue_size=10)
        self.finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10, latch=True)

        self.maxSteer = float(rospy.get_param("/maxSteer"))

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def run(self) -> None:
        """
        Run Static A
        """

        if not self.started:
            rospy.loginfo("Waiting for driving flag")
            return

        rospy.sleep(2)
        rospy.loginfo("publishing max steer")
        self.statePub.publish(1)
        self.steerPub.publish(-self.maxSteer)
        rospy.sleep(10)
        rospy.loginfo("publishing max steer")
        self.steerPub.publish(self.maxSteer)
        rospy.sleep(10)
        rospy.loginfo("publishing 0 steer")
        self.steerPub.publish(0)
        rospy.sleep(10)

        timeStart = time.time()
        while time.time() - timeStart < 10:
            vel = 2 * np.pi * 200 * 0.253 / 60 * (time.time() - timeStart)
            self.velPub.publish(vel)
            rospy.sleep(0.1)

        self.velPub.publish(0)

        rospy.sleep(2)
        self.velPub.publish(0)
        rospy.sleep(2)

        self.statePub.publish(2)  # 2 is for finished
        self.finishPub.publish(True)  # 1 is for finished
        rospy.sleep(10)
