#!/usr/bin/python3
"""
.

"""
import rospy
from std_msgs.msg import Bool
from asurt_msgs.msg import RoadState
from tf_helper.StatusPublisher import StatusPublisher


ROAD_STATE = RoadState()


def roadStateCallback(msg: RoadState) -> None:
    """
    Callback to retreive the road state
    """
    global ROAD_STATE  # pylint: disable=global-statement
    ROAD_STATE = msg.data


def main() -> None:
    """
    track drive
    """

    global ROAD_STATE  # pylint: disable=global-statement, global-variable-not-assigned
    rospy.init_node("autocross_finisher ")

    isFinishedTopic = rospy.get_param("/finisher/is_finished")
    roadStateTopic = rospy.get_param("/slam/road_state")

    isFinishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10)

    rospy.Subscriber(roadStateTopic, RoadState, roadStateCallback)

    status = StatusPublisher("/status/autocross")
    status.starting()
    status.ready()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status.running()
        if ROAD_STATE.laps >= 1:
            isFinishPub.publish(True)

        rate.sleep()


if __name__ == "__main__":
    ROAD_STATE = RoadState()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
