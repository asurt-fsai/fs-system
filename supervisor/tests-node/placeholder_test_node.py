#!/usr/bin/python3
"""
.

"""
import rospy
from tf_helper.StatusPublisher import StatusPublisher


def main() -> None:
    """
    Main function.
    """
    rospy.init_node("placeholder_test_node")
    statusPublisher = StatusPublisher("/status/placeholder_test_node")
    statusPublisher.starting()
    statusPublisher.ready()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        statusPublisher.running()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
