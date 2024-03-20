# pylint: disable=all
# mypy: ignore-errors
from rclpy.node import Node
import rclpy
from .MarkerViz import MarkerViz


class TestMarkerViz(Node):
    def __init__(self):
        super().__init__("test")
        test = MarkerViz(self, 1, 1, 1)
        test.createDeleteMsg("test")
        test.getConeColor(1)


def main(args=None):
    rclpy.init(args=args)
    test2 = TestMarkerViz()

    rclpy.spin(test2)
    rclpy.shutdown()
