"""
StatusPublisher class to provide an easy way to publish a heartbeat
"""
from typing import List
from threading import Lock


from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from asurt_msgs.msg import NodeStatus


class StatusPublisher:
    """
    Class used to publish a heartbeat.
    Each node should have one and only one StatusPublisher
    The following are the statuses that can be published:
        - Starting
        - Ready
        - Running
        - Error

    Parameters
    ----------
    topicName:str
        Name of the topic to publish the heartbeat on
        IMPORTANT: must be unique, otherwise an exception will be thrown
    """

    topicNamesCreated: List[str] = []

    def __init__(self, topicName: str, nodeObject: Node) -> None:
        if topicName in StatusPublisher.topicNamesCreated:
            raise ValueError(
                "StatusPublisher: Topic name already exists, \
                 can't publish a heartbeat on the same topic twice"
            )
        latchingQOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.nodeObject = nodeObject
        self.publisher = self.nodeObject.create_publisher(
            topic=topicName, msg_type=NodeStatus, qos_profile=latchingQOS
        )
        StatusPublisher.topicNamesCreated.append(topicName)
        self.lock = Lock()

    def baseMessage(self) -> NodeStatus:
        """
        Creates a base NodeStatus message with a timestamp

        Returns
        -------
        NodeStatus
            Base NodeStatus message
        """
        msg = NodeStatus()
        msg.header.stamp = self.nodeObject.get_clock().now().to_msg()
        return msg

    def starting(self) -> None:
        """
        Publishes a NodeStatus message with the state "Starting"
        """
        with self.lock:
            msg = self.baseMessage()
            msg.status = 0
            self.publisher.publish(msg)

    def ready(self) -> None:
        """
        Publishes a NodeStatus message with the state "Ready"
        """
        with self.lock:
            msg = self.baseMessage()
            msg.status = 1
            self.publisher.publish(msg)

    def running(self) -> None:
        """
        Publishes a NodeStatus message with the state "Running"
        """
        with self.lock:
            msg = self.baseMessage()
            msg.status = 2
            self.publisher.publish(msg)

    def error(self, errMsg: str) -> None:
        """
        Publishes a NodeStatus message with the state "Error"

        Parameters
        ----------
        errMsg: str
            Error message to include with the NodeStatus message
        """
        with self.lock:
            msg = self.baseMessage()
            msg.status = 3
            msg.message = errMsg
            self.publisher.publish(msg)
