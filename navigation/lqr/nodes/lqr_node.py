#!/usr/bin/env python3
"""
lqr handler to integrate the following modules:
    - lqrPrepareTrack
    - lqr_optimize_track
"""
import rospy
from nav_msgs.msg import Path
from lqr import Track

TRACK_WIDTH = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN = rospy.get_param("/navigation/lqr/handler/safety_margin")


def main() -> None:
    """
    Main Function of the lqrHandler
    """
    rospy.init_node("lqr")
    rospy.loginfo("Waiting for Message")
    message = rospy.wait_for_message("/waypoints", Path, timeout=None)
    rospy.loginfo("Recieved Message")

    track = Track(message)

    # Publish the raceline
    raceLinePub = rospy.Publisher("/lqr/raceline", Path, queue_size=10)
    upperBoundPub = rospy.Publisher("/lqr/upperBound", Path, queue_size=10)
    lowerBoundPub = rospy.Publisher("/lqr/lowerBound", Path, queue_size=10)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Publishing waypoints")
            raceLinePub.publish(track.optimized.racelineMsg)
            upperBoundPub.publish(track.optimized.bounds.upperBoundMsg)
            lowerBoundPub.publish(track.optimized.bounds.lowerBoundMsg)
            rospy.sleep(5)
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down")
            break
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            break


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
