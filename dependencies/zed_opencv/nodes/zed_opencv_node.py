#!/usr/bin/python3
"""
Main ros node for the lidar pipeline used to detect cones
"""
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("zed_opencv")
    videoCapture = cv2.VideoCapture(0)
    bridge = CvBridge()
    imgLeftPub = rospy.Publisher("/zed/zed_node/left/image_rect_color", Image, queue_size=10)
    imgRightPub = rospy.Publisher("/zed/zed_node/right/image_rect_color", Image, queue_size=10)

    # Set image width and height and fps
    width = 1344
    videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
    videoCapture.set(cv2.CAP_PROP_FPS, 30)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        _, frame = videoCapture.read()
        leftImage = bridge.cv2_to_imgmsg(frame[:, : width // 2], "rgb8")
        rightImage = bridge.cv2_to_imgmsg(frame[:, width // 2 :], "rgb8")
        imgLeftPub.publish(leftImage)
        imgRightPub.publish(rightImage)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
