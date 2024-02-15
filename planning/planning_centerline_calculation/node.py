#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from full_pipeline.full_pipeline import PathPlanner

class PlanningNode(Node):

    def __init__(self):
        super().__init__("planning_node")
        self.get_logger().info("Path Planner instantiated...")
        self.path = None
        self.cones_positions = None
        self.cones_colors = None
        self.car_position = None
        self.car_direction = None
        self.subscriber1 = self.create_subscription(
            str, "/topic", self.receive_from_perception, 10
        )
        self.subscriber2 = self.create_subscription(
            str, "/topic", self.receive_from_localization, 10
        )
        self.publisher = self.create_publisher(
            str, "/topic2", 10
        )


    def receive_from_perception(self, msg: str):
        #get cones_colors
        return
    
    def receive_from_localization(self, msg: str):
        #get cones_positions, car_position, car_direction
        return
    
    def send_to_control(self):
        pathPlanner = PathPlanner()
        path = pathPlanner.calculatePathInGlobalFrame(
            vehiclePosition= self.car_position,
            vehicleDirection= self.car_direction,
            cones= self.cones_positions,

        )
        return path


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
