# pylint: disable=all
# mypy: ignore-errors
from .StatusPublisher import StatusPublisher
from rclpy.node import Node
import rclpy

# node = Node("node name")


class TestStatusPublisher(Node):
    def __init__(self):
        super().__init__("test")
        # self.status_publisher_test = StatusPublisher("test", node)
        test = StatusPublisher("/test", self)
        test.starting()


# test2 = TestStatusPublisher()


def main(args=None):
    rclpy.init(args=args)
    test2 = TestStatusPublisher()

    rclpy.spin(test2)
    rclpy.shutdown()
