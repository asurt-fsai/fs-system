# pylint: disable=all
# mypy: ignore-errors
from TFHelper import TFHelper
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from tf2_ros import TransformBroadcaster

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion


class TestTFHelper(Node):
    def __init__(self):
        super().__init__("test")
        # self.tf_helper_test = TFHelper("test", node)
        self.tfhelp = TFHelper(self)

        self.msg = Path()
        self.msg.header.frame_id = "local"
        pose = PoseStamped()
        pose.header.frame_id = "local"
        pose.pose.position.x = 8.0
        pose.pose.position.y = 6.0
        pose.pose.position.z = 9.0
        pose.pose.orientation.w = 0.1
        self.msg.poses.append(pose)
        # self.msg.header.frame_id = "map"
        print("Msg Before Transform: " + str(self.msg.poses[0].pose.position.y))
        # print(self.tfhelp.transformMsg(msg, "map"))

        self.transMsg = TransformStamped()
        self.transMsg.transform.translation = Vector3(x=1.0, y=2.0, z=3.0)
        self.transMsg.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.1)
        self.transMsg.header.frame_id = "map"
        self.transMsg.child_frame_id = "local"

        self.broadcaster = TransformBroadcaster(self)

        self.create_timer(1, self.test)

    def test(self):
        self.msg = Path()
        self.msg.header.frame_id = "local"
        pose = PoseStamped()
        pose.header.frame_id = "local"
        pose.pose.position.x = 8.0
        pose.pose.position.y = 6.0
        pose.pose.position.z = 9.0
        pose.pose.orientation.w = 5.0
        self.msg.poses.append(pose)
        # Lazm acreate new object for each message 3shan el tf ye7sal feha

        # self.broadcaster.sendTransform([0, 0, 0], [0, 0, 0, 1], self.get_clock().now().to_msg(), "map", "odom")
        # print(self.tfhelp.getTransform("map", "local")) #working properly
        # print(self.tfhelp.transformArr3d("map", "odom"))
        self.broadcaster.sendTransform(self.transMsg)  # working properly
        # print(self.msg)
        self.msg = self.tfhelp.transformMsg(self.msg, "map")  # not working
        # print("Msg After Transform: " + str(self.msg.poses[0].pose.position.y))
        # print("Test2")


def main(args=None):
    rclpy.init(args=args)
    node = TestTFHelper()
    node.test()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
