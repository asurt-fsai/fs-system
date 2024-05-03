import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
class nodetest(Node):
    def __init__(self):
        super().__init__("testingSLam")

        self.publisher = self.create_publisher(Path, "/testPath", 10)    

        self.create_timer(1, self.pub)

    def pub(self):

        path = Path()
        pose1 = PoseStamped()
        pose2 = PoseStamped()

        pose1.pose.position.x = 0.0
        pose1.pose.position.y = 0.0
        pose1.pose.position.z = 0.0

        pose2.pose.position.x = 4.0
        pose2.pose.position.y = 5.0
        pose2.pose.position.z = 0.0

        path.poses.append(pose1)
        path.poses.append(pose2)

        path.header.frame_id = "camera"
        self.publisher.publish(path)


def main(args = None):
    rclpy.init(args=args)

    test = nodetest()
    rclpy.spin(test)


if __name__ == "__main__":
    main()