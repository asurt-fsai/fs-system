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


def main():
    rclpy.init()
    fastSlamNode = FastSlamNode()
    rclpy.spin(fastSlamNode)
    fastSlamNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
