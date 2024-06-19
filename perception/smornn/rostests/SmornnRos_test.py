#!/usr/bin/env python3
"""
Ros tests for the SmornnRos wrapper class
"""
import sys
import time
import unittest
from typing import Generator, Any

import rospy
import rostest

# from smornn import SmornnRos

PKG = "smornn"
NAME = "test_smornn"


def msgStatus(topicName: str) -> Generator[Any, Any, Any]:
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

    rospy.init_node("test_smornn_ros_wrapper", anonymous=True)
    rospy.Subscriber(topicName, Any, callback)

    yield

    timeout = time.time() + 1.0

    while not rospy.is_shutdown() and msgReceived is None and timeout > time.time():
        time.sleep(0.1)

    yield msgReceived


class TestSmornnRosWrapper(unittest.TestCase):
    """
    Test cases for the SmornnRosWrapper class
    """

    def testBagStatistics(self) -> None:
        """
        Tests publishing a starting message
        """


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestSmornnRosWrapper, sys.argv)
