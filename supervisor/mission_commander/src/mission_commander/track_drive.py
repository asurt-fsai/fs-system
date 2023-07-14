#!/usr/bin/python3
"""
TrackDrive
"""


import rospy
from std_msgs.msg import Bool
from asurt_msgs.msg import RoadState


class TrackDrive:
    """
    TrackDrive
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self, isFinishedTopic: str) -> None:
        rospy.loginfo("Starting TrackDrive")

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
        Run TrackDrive
        """

        if self.roadState.laps >= 10:
            self.finishPub.publish(True)
