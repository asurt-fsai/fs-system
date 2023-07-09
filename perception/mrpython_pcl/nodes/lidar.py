#!/usr/bin/python3
"""
Main ros node for the lidar pipeline used to detect cones
"""
import rospy

from mrpython_pcl.ros.Builders import Builder
from tf_helper.StatusPublisher import StatusPublisher


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("mr_lidar")
    status = StatusPublisher("/status/lidar")
    status.starting()

    builder = Builder()
    lidar = builder.buildPipeline()

    # Publish heartbeat to show the module is ready
    status.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        try:
            out = lidar.run()
        except Exception as exp:  # pylint: disable=broad-except
            rospy.logwarn("Lidar Pipeline failed: " + str(exp))
            continue
        if out is None:
            continue

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
