#!/usr/bin/python3
"""
Autocross
"""

import rospy
from std_msgs.msg import Bool
from asurt_msgs.msg import RoadState


class Autocross:
    """
    Autocross
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self, isFinishedTopic: str) -> None:
        rospy.loginfo("Starting Autocross")

        self.roadState = RoadState()

        roadStateTopic = rospy.get_param("/slam/road_state")
        rospy.Subscriber(roadStateTopic, RoadState, self.roadStateCallback)

        self.finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10, latch=True)

    def roadStateCallback(self, msg: RoadState) -> None:
        """
        Callback to retreive the road state
        """
        self.roadState = msg.data

    def run(self) -> None:
        """
        Run Autocross
        """

        if self.roadState.laps >= 1:
            self.finishPub.publish(True)
