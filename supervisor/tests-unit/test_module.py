#!/usr/bin/env python3
"""
Ros tests for the Module class
"""
import sys
import unittest
from typing import Generator, Any
import rospy
import rosnode
import rostest
from asurt_msgs.msg import NodeStatus
from supervisor.helpers import Module  # type: ignore[attr-defined]

PKG = "supervisor"
NAME = "test_module"
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


class TestSupervisorModule(unittest.TestCase):
    """
    Test cases for the TestSupervisorModule class
    """

    def testLaunchShutdownRestart(self) -> None:
        """
        Tests launching and shutting down then restarting a node
        """
        rospy.init_node("test_supervisor_module", anonymous=True)

        statusTopic = "/status/placeholder_test_node"
        module = Module("supervisor", "placeholder_test_node.launch", statusTopic)
        module.launch()
        rospy.sleep(2)

        statusCallback = msgStatus(statusTopic, NodeStatus)
        next(statusCallback)
        nodeStatus = next(statusCallback)
        self.assertIsNotNone(nodeStatus)
        self.assertIn("/placeholder_test_node", rosnode.get_node_names())

        module.shutdown()
        rospy.sleep(2)
        statusCallback = msgStatus(statusTopic, NodeStatus)
        next(statusCallback)
        nodeStatus = next(statusCallback)
        self.assertIsNone(nodeStatus)
        self.assertNotIn("/placeholder_test_node", rosnode.get_node_names())

        module.restart()
        rospy.sleep(2)
        statusCallback = msgStatus(statusTopic, NodeStatus)
        next(statusCallback)
        nodeStatus = next(statusCallback)
        self.assertIsNotNone(nodeStatus)
        self.assertIn("/placeholder_test_node", rosnode.get_node_names())

        rospy.sleep(10)
        heartbeatRate = module.rate
        self.assertLess(abs(heartbeatRate - 10), 0.5)
        module.shutdown()

        # Test module with no heartbeat
        module = Module("supervisor", "placeholder_test_node.launch", None)
        module.launch()
        rospy.sleep(2)
        self.assertIn("/placeholder_test_node", rosnode.get_node_names())

        module.shutdown()
        rospy.sleep(2)
        self.assertNotIn("/placeholder_test_node", rosnode.get_node_names())

        module.restart()
        rospy.sleep(2)
        self.assertIn("/placeholder_test_node", rosnode.get_node_names())


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestSupervisorModule, sys.argv)
