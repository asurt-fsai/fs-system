#!/usr/bin/python3
"""
Main ros node for the lidar pipeline used to detect cones
"""
import rospy

# pylint: disable=import-error, no-name-in-module
from mrpython_pcl.ros.Builders import buildPipeline
from tf_helper.StatusPublisher import StatusPublisher


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("mr_lidar")
    status = StatusPublisher("/status/lidar")
    status.starting()

    lidar = buildPipeline()

    # Create a ROS subscriber for the input point cloud
    status.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        out = lidar.run()
        if out is None:
            continue

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass