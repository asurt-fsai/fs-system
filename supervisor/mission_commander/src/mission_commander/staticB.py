#!/usr/bin/python3
"""
Static A
"""

import rospy
import numpy as np

from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger, TriggerRequest


class StaticB:
    """
    Static B
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self, velTopic: str, ebsTopic: str, isFinishedTopic: str, stateTopic: str) -> None:
        rospy.loginfo("Starting Static B")

        self.drivingFlag = False
        self.started = False

        rospy.wait_for_service(ebsTopic)
        self.ebsService = rospy.ServiceProxy(ebsTopic, Trigger)
        self.velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        self.statePub = rospy.Publisher(stateTopic, Float32, queue_size=10)
        self.finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10, latch=True)

        self.maxSteer = float(rospy.get_param("/maxSteer"))

        rospy.loginfo("Starting Static B")

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def run(self) -> None:
        """
        Run Static B
        """

        if not self.started:
            rospy.loginfo("Waiting for driving flag")
            return

        rospy.sleep(2)
        vel = 2 * np.pi * 50 * 0.253 / 60  # 50 rpm
        self.velPub.publish(vel)
        rospy.sleep(10)

        ebs = TriggerRequest()
        self.ebsService(ebs)

        self.statePub.publish(2)  # 2 is for finished
        self.finishPub.publish(True)  # 1 is for finished
