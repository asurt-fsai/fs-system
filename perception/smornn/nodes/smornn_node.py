#!/usr/bin/python3
"""
Main ros node for the smornn pipeline used to combine detections from lidar and smoreo
"""
import rospy
from visualization_msgs.msg import MarkerArray

from asurt_msgs.msg import LandmarkArray
from tf_helper.StatusPublisher import StatusPublisher
from tf_helper.MarkerViz import MarkerViz

from smornn.SmornnRos import SmornnRos


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("smornn")
    status = StatusPublisher("/status/smornn")
    status.starting()

    publishTopic = rospy.get_param("/perception/smornn/detected")
    markerTopic = rospy.get_param("/perception/smornn/detected_markers")
    publishers = {
        "detected": rospy.Publisher(publishTopic, LandmarkArray, queue_size=1),
        "detected_markers": rospy.Publisher(markerTopic, MarkerArray, queue_size=1),
    }

    lidarHeight = rospy.get_param("/physical/lidar_height")
    coneHeight = rospy.get_param("/physical/cone_height")
    coneRadius = rospy.get_param("/physical/cone_radius")
    markerViz = MarkerViz(coneRadius, coneHeight, -1 * lidarHeight + coneHeight / 2)

    minDistNeighbor = rospy.get_param("/perception/smornn/min_dist_neighbor")
    frameId = rospy.get_param("/perception/smornn/frame_id")

    smornn = SmornnRos(publishers, markerViz, frameId, minDistNeighbor)

    lidarTopic = rospy.get_param("/perception/lidar/detected")
    smoreoTopic = rospy.get_param("/perception/smoreo/detected")
    rospy.Subscriber(lidarTopic, LandmarkArray, smornn.lidarCallback)
    rospy.Subscriber(smoreoTopic, LandmarkArray, smornn.smoreoCallback)

    # Publish heartbeat to show the module is ready
    status.ready()

    # Main loop
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
        out = smornn.run()
        if out is None:
            continue

        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
