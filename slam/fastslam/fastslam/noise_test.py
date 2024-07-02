from rclpy.node import Node
import rclpy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


class Test(Node):
    def __init__(self):
        super().__init__("test_node")
        # generate random points

        self.nLandmarks = 100
        self.landmarks = np.array([])
        for i in range(self.nLandmarks):
            i = i
            self.landmarks = np.append(self.landmarks, np.random.rand(2) * 50)

        self.landmarks = self.landmarks.reshape(self.nLandmarks, 2)
        # create publisher
        self.markerPub = self.create_publisher(MarkerArray, "landmarks_marker", 10)
        # create timer
        self.timer = self.create_timer(0.1, self.timerClbk)
        self.i = 0

    def timerClbk(self):
        # create message
        msg = MarkerArray()
        for i in range(self.nLandmarks):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD

            marker.pose.position.x = self.landmarks[i, 0] + np.random.normal(0, 0.1)
            marker.pose.position.y = self.landmarks[i, 1] + np.random.normal(0, 0.1)

            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            msg.markers.append(marker)
        # publish message
        self.markerPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()
