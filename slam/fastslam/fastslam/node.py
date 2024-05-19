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
from message_filters import Subscriber, ApproximateTimeSynchronizer


class FastSlamNode(Node):
    def __init__(self):
        super().__init__("fastslam_node")
        self.landmarks = np.array([])
        self.observations = np.array([])

        self.markerPub = self.create_publisher(MarkerArray, "landmarks_marker_hung", 10)
        self.fastSlam = FastSLAM()

        self.tss = ApproximateTimeSynchronizer({Subscriber(self,LandmarkArray,"/Landmarks/Observed"),Subscriber(self,Odometry,"/carmaker/Odometry")},10,0.1,True)
        self.tss.registerCallback(self.callback)

    def callback(self, landmarks:LandmarkArray, odometry:Odometry):
        print(landmarks.landmarks[0].position)
        print(odometry.pose.pose.position)


    def callback(self, msg: LandmarkArray, odom: Odometry):
        self.observations = np.array([])
        for marker in msg.markers:
            self.observations = np.append(
                self.observations, [marker.pose.position.x, marker.pose.position.y]
            ).reshape(-1, 2)
        if self.landmarks.size == 0:
            self.landmarks = self.observations
        hung = HungarianAlg(self.observations, self.landmarks)
        self.landmarks, associatedObservations = hung.solve()

        # convert observations to range-bearing
        observations = []
        for obs in associatedObservations:
            zObs = complex(obs[0], obs[1])
            pose = complex(odom.pose.pose.position.x, odom.pose.pose.position.y)
            r = abs(zObs - pose)
            b = np.arctan2(obs[1] - odom.pose.pose.position.y, obs[0] - odom.pose.pose.position.x) - odom.pose.pose.orientation.z
            observations.append([r, b, obs[2]])
        

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
