#!/usr/bin/env python3
"""
Ros tests for the StatusPublisher class
"""
import sys
import time
import unittest
from typing import Generator, Any

import rospy
import rostest

from asurt_msgs.msg import NodeStatus
from tf_helper.StatusPublisher import (
    StatusPublisher,
)

PKG = "tf_helper"
NAME = "test_status_publisher"


def msgStatus(topicName: str) -> Generator[NodeStatus, Any, Any]:
    """
    Waits for a message to be received and returns the status

    Yields
    -------
    Optional[int]
        Message ID received
    """
    msgReceived = None

    def callback(msg: NodeStatus) -> None:
        nonlocal msgReceived
        msgReceived = msg

    rospy.init_node("test_status_publisher", anonymous=True)
    rospy.Subscriber(topicName, NodeStatus, callback)

    yield

    timeout = time.time() + 1.0

    while not rospy.is_shutdown() and msgReceived is None and timeout > time.time():
        time.sleep(0.1)

    yield msgReceived


class TestStatusPublisher(unittest.TestCase):
    """
    Test cases for the StatusPublisher class
    """

    def testPublishStarting(self) -> None:
        """
        Tests publishing a starting message
        """
        iterator = msgStatus("starting_message")
        next(iterator)
        statusPublisher = StatusPublisher("starting_message")
        statusPublisher.starting()
        receivedValue = next(iterator)

        self.assertTrue(receivedValue.status == 0)

    def testPublishReady(self) -> None:
        """
        Tests publishing a ready message
        """
        iterator = msgStatus("ready_message")
        next(iterator)
        statusPublisher = StatusPublisher("ready_message")
        statusPublisher.ready()
        receivedValue = next(iterator)

        self.assertTrue(receivedValue.status == 1)

    def testPublishRunning(self) -> None:
        """
        Tests publishing a running message
        """
        iterator = msgStatus("running_message")
        next(iterator)
        statusPublisher = StatusPublisher("running_message")
        statusPublisher.running()
        receivedValue = next(iterator)

        self.assertTrue(receivedValue.status == 2)

    def testPublishError(self) -> None:
        """
        Tests publishing an error message
        """
        iterator = msgStatus("error_message")
        next(iterator)
        statusPublisher = StatusPublisher("error_message")
        statusPublisher.error("message content")
        receivedValue = next(iterator)

        self.assertTrue(receivedValue.status == 3)
        self.assertTrue(receivedValue.message == "message content")

    def testNoTwoTopicsSameName(self) -> None:
        """
        Should throws a ValueError when trying to create two
        status publishers with the same topic name
        """
        StatusPublisher("same_topic_name")
        with self.assertRaises(ValueError):
            StatusPublisher("same_topic_name")


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestStatusPublisher, sys.argv)
