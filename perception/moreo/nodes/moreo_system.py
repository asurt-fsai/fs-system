#!/usr/bin/env python3
"""
Main ros node for the moreo pipeline used to detect cones
"""
import rospy
from tf_helper.StatusPublisher import StatusPublisher
from moreo.moreoPipeline import MoreoSystem


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("moreo")
    status = StatusPublisher("/status/moreo")
    status.starting()

    moreo = MoreoSystem()

    # Publish heartbeat to show the module is ready
    status.ready()

    moreo.startMoreo()
    # Publish heartbeat to show the module is running
    status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
