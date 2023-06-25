#!/usr/bin/python3
"""
ros wrapper for the depth from zed module
"""

import rospy
from tf_helper.StatusPublisher import StatusPublisher
from zed.conesFromDisparity import ConesFromDisparity


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("zed_cones")
    status = StatusPublisher("/status/zed_cones")

    status.starting()

    pixelsToConsider = rospy.get_param("/zed/pixels_to_consider", 100)

    conesFromDisparity = ConesFromDisparity(pixelsToConsider=int(pixelsToConsider))

    disparityTopic = rospy.get_param(
        "/zed/disparity_topic", "/zed/zed_node/disparity/disparity_image"
    )
    bboxTopic = rospy.get_param("/darknet_ros/bounding_boxes_topic", "/darknet_ros/bounding_boxes")
    conesTopic = rospy.get_param("/zed/cones_topic", "zed/detected_cones")
    cameraInfoTopic = rospy.get_param("/zed/camera_info_topic", "/zed/zed_node/left/camera_info")

    conesFromDisparity.start(disparityTopic, bboxTopic, conesTopic, cameraInfoTopic)

    status.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        out = conesFromDisparity.run()
        if out is None:
            continue
        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
