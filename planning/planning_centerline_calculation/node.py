#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dependencies.asurt_msgs.msg.LandmarkArray.msg import LandmarkArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from utils.cone_types import ConeTypes
import numpy as np

from full_pipeline.full_pipeline import PathPlanner

class PlanningNode(Node):

    def __init__(self):
        super().__init__("planning_node")
        self.get_logger().info("Path Planner instantiated...")
        self.path = None
        self.cones = None
        self.car_position = None
        self.car_direction = None
        self.subscriber1 = self.create_subscription(
            LandmarkArray, "/topic1", self.receive_from_perception, 10
        )
        self.subscriber2 = self.create_subscription(
            Odometry, "/topic2", self.receive_from_localization, 10
        )
        self.publisher = self.create_publisher(
            PoseArray, "/topic3", 10
        )


    def receive_from_perception(self, msg: LandmarkArray):
        #get cones_colors, cones_positions
        self.cones = [np.zeros((0, 2)) for _ in ConeTypes]
        for landmark in msg.landmarks:
            if landmark.identifier == 0:
                self.cones[ConeTypes.BLUE] = np.vstack((self.cones[ConeTypes.BLUE], landmark.position))
            elif landmark.identifier == 1:
                self.cones[ConeTypes.YELLOW] = np.vstack((self.cones[ConeTypes.YELLOW], landmark.position))
            else:
                self.cones[ConeTypes.UNKNOWN] = np.vstack((self.cones[ConeTypes.UNKNOWN], landmark.position))
            
        self.send_to_control()

    
    def receive_from_localization(self, msg: Odometry):
        #get car_position, car_direction
        pose = msg.pose.pose
        self.car_position = [pose.position.x, pose.position.y]
        self.car_direction = [pose.orientation.x, pose.orientation.y]

    
    def send_to_control(self):
        pathPlanner = PathPlanner()
        self.path = pathPlanner.calculatePathInGlobalFrame(
            vehiclePosition= self.car_position,
            vehicleDirection= self.car_direction,
            cones= self.cones
        )
        if self.path is not None:
            pose_array = PoseArray()
            for data_point in self.path:
                pose = Pose()
                pose.position.x = data_point[0]
                pose.position.y = data_point[1]
                pose_array.poses.append(pose)

            self.publisher.publish(pose_array)
            self.get_logger().info('Path Sent...')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
