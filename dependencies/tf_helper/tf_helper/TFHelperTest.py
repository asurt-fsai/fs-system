# pylint: disable=all
# mypy: ignore-errors
from TFHelper import TFHelper
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TestTFHelper(Node):
    def __init__(self):
        super().__init__("test")
        # self.tf_helper_test = TFHelper("test", node)
        self.tfhelp = TFHelper(self)
        msg = Path()
        print("Test")
        print(self.tfhelp.transformMsg(msg, "map"))

    def test(self):
        print(self.tfhelp.getTransform("map", "odom"))
        print(self.tfhelp.transformArr3d("map", "odom"))
        print("Test2")


def main(args=None):
    rclpy.init(args=args)
    node = TestTFHelper()
    node.test()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
