import rclpy
import random
import time
import numpy as np
from rclpy.node import Node
from asurt_msgs.msg import LandmarkArray, Landmark
from .hungarian import HungarianAlg
from icecream import ic
from .fastslam import FastSLAM
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry


class FastSlamNode(Node):
    def __init__(self):
        super().__init__("fastslam_node")
        self.landmarks = np.array([])
        self.observations = np.array([])
        self.landmarksSub = self.create_subscription(
            MarkerArray, "landmarks_marker", self.landmarkCallback, 10
        )
        self.odomSub = self.create_subscription(
            Odometry, "/carmaker/Odometry", self.odomCallback, 10
        )
        self.markerPub = self.create_publisher(MarkerArray, "landmarks_marker_hung", 10)
        self.fastSlam = FastSLAM()

    def odomCallback(self, msg: Odometry):
        odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0])
        self.fastSlam.estimatePose(odom)

    def landmarkCallback(self, msg: MarkerArray):
        start = time.time()
        self.observations = np.array([])
        for marker in msg.markers:
            self.observations = np.append(
                self.observations, [marker.pose.position.x, marker.pose.position.y]
            ).reshape(-1, 2)
        if self.landmarks.size == 0:
            self.landmarks = self.observations
        hung = HungarianAlg(self.observations, self.landmarks)
        self.landmarks = hung.solve()
        hung.printResults()
        end = time.time()
        print("Size:", hung.n, hung.m)
        ic(end - start)

        mrkArr = MarkerArray()
        for i in range(self.landmarks.shape[0]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = self.landmarks[i, 0]
            marker.pose.position.y = self.landmarks[i, 1]
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            mrkArr.markers.append(marker)

        self.markerPub.publish(mrkArr)


def main():
    rclpy.init()
    fastSlamNode = FastSlamNode()
    rclpy.spin(fastSlamNode)
    fastSlamNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
