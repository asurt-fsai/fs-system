#!/usr/bin/env python3
"""
This node calculates the centerline of the track
"""
import rospy
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from centerline_calc import Centerline


def main() -> None:
    """
    Main Function of the lqrHandler
    """
    rospy.init_node("centerline_calc")
    rospy.loginfo("Waiting for Message")
    message = rospy.wait_for_message("/cones", MarkerArray, timeout=None)
    rospy.loginfo("Recieved Message")

    centerline = Centerline(conesMessage=message)
    centerlinePathMsg = centerline.centerlineToMsg(centerline.centerline)
    centerlinePub = rospy.Publisher("/centerline", Path, queue_size=10)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Publishing centerline")
            centerlinePub.publish(centerlinePathMsg)
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
        rospy.loginfo("Shutting down")
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
