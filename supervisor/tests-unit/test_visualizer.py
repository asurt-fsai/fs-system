#!/usr/bin/env python3
"""
Ros tests for the Visualizer class
"""
import sys
import unittest
from typing import Generator, Any
import rospy
import rostest
from visualization_msgs.msg import MarkerArray

PKG = "supervisor"
NAME = "test_visualizer"
# pylint: disable = duplicate-code


def msgStatus(topicName: str, msgType: Any) -> Generator[Any, Any, Any]:
    """
    Waits for a message to be received and returns the status

    Yields
    -------
    Optional[int]
        Message ID received
    """
    msgReceived = None

    def callback(msg: Any) -> None:
        nonlocal msgReceived
        msgReceived = msg

    rospy.Subscriber(topicName, msgType, callback)

    yield

    timeout = rospy.Time.now().to_sec() + 5.0

    while not rospy.is_shutdown() and msgReceived is None and timeout > rospy.Time.now().to_sec():
        rospy.sleep(0.1)

    yield msgReceived


class TestSupervisorVisualizer(unittest.TestCase):
    """
    Test cases for the Visualizer class
    """

    def testVisualizerTopics(self) -> None:
        """
        Ensures the visualizer topics are created and publishing as expected
        according to the visualizer_test_node.py script
        Button is tested manually by running rosrun supervisor visualizer_test_node.py
        """
        rospy.init_node("test_visualizer_topics")
        textMsg = msgStatus("/visualizer/text", MarkerArray)
        next(textMsg)
        text = next(textMsg)
        self.assertIsNotNone(text)
        assert text is not None
        self.assertEqual(text.markers[1].text, "Hello World")


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestSupervisorVisualizer, sys.argv)
