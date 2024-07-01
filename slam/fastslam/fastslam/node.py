import rclpy
import random
import time
import numpy as np
from rclpy.node import Node
from asurt_msgs.msg import LandmarkArray, Landmark
from fastslam import FastSLAM
from fastslam import Particle as Particle
from icecream import ic
from utils.utils import angleToPi
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer


class FastSlamNode(Node):
    def __init__(self):
        super().__init__("fastslam_node")
        self.landmarks = np.array([]).reshape(-1, 4)
        self.observations = np.array([]).reshape(-1, 3)

        self.markerPub = self.create_publisher(MarkerArray, "landmarks_marker_hung", 10)
        self.fastSlam = FastSLAM(10)

        self.tss = ApproximateTimeSynchronizer({Subscriber(self,LandmarkArray,"/Landmarks/Observed"),Subscriber(self,Odometry,"/carmaker/Odometry")},10,0.1,True)
        self.tss.registerCallback(self.callback)
        self.time = self.get_clock().now()

    def callback(self, landmarks:LandmarkArray, odometry:Odometry):
        print(landmarks.landmarks[0].position)
        print(odometry.pose.pose.position)


    def callback(self, msg: LandmarkArray, odom: Odometry):
        for i in range(len(msg.landmarks)):
            self.observations = np.vstack([self.observations, [msg.landmarks[i].position.x, msg.landmarks[i].position.y, msg.landmarks[i].type]])
        
        controlAction = np.array([odom.twist.twist.linear.x, odom.twist.twist.angular.z])
        dt = (self.get_clock().now() - self.time).nanoseconds / 1e9
        self.time = self.get_clock().now()
        self.fastSlam.update(controlAction, self.observations, dt)
        self.landmarks = self.fastSlam.maxWeightParticle.landmarks

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
