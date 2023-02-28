"""
Main ros node for the smoreo tuner
"""
#!/usr/bin/python3

import rospy
from tf_helper.StatusPublisher import StatusPublisher
from smoreo.tuner.tunerServer import Tuner


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("tuner")
    tunerStatus = StatusPublisher("/status/tuner")

    tunerStatus.starting()

    tuner = Tuner()
    tuner.start()

    tunerStatus.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        tuner.visualizeCutOff()
        # Publish heartbeat to show the module is running
        tunerStatus.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
