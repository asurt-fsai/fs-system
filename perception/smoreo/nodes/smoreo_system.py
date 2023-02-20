#!/usr/bin/python3
"""
Main ros node for the smoreo pipeline used to detect cones
"""
import rospy
from tf_helper.StatusPublisher import StatusPublisher
from smoreo.smoreoRosWrapper import SmoreoRosWrapper


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("smoreo")
    status = StatusPublisher("/status/smoreo")

    status.starting()

    smoreo = SmoreoRosWrapper()
    smoreo.start()

    status.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        out = smoreo.run()
        if out is None:
            continue
        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
