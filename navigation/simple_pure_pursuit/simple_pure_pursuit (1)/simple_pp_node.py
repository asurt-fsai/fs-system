#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
#import rospy
from nav_msgs.msg import Path 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from .simple_pure_pursuit import SimplePurePursuit  # type: ignore[attr-defined]
from ackermann_msgs.msg import AckermannDrive
import rclpy
from rclpy.node import Node

class SimplePurePursuitNode(Node):

    def __init__(self):
        super().__init__('simple_pure_pursuit_node')
        self.controller = SimplePurePursuit()
        self.rate = self.create_rate(1)#self.controlRate)
        
        self.markerPub = self.create_publisher(Marker, '/marker_viz', 10)
        self.steeringPub = self.create_publisher(AckermannDrive, '/steer', 10)
        self.create_subscription(Path, '/topic3', self.controller.add, 10)  #waypoints      
        self.create_timer(1, self.loop)

    def loop(self):
        delta, ind = self.controller.purepursuitSteercontrol()
        steerMsg = AckermannDrive()
        steerMsg.steering_angle = delta
        steerMsg.speed = 1.0
        #steerMsg.data = delta
        self.steeringPub.publish(steerMsg)
        # self.rate.sleep()

        if len(self.controller.xList) > 0:
            vizPoint = Marker()
            vizPoint.header.frame_id = "map"

            
            vizPoint.ns = "pure_pursuit"
            vizPoint.id = 0
            vizPoint.type = Marker.SPHERE
            vizPoint.action = Marker.ADD
            vizPoint.pose.position.x = self.controller.xList[ind]
            vizPoint.pose.position.y = self.controller.yList[ind]
            vizPoint.pose.position.z = 0.0
            vizPoint.pose.orientation.x = 0.0
            vizPoint.pose.orientation.y = 0.0
            vizPoint.pose.orientation.z = 0.0
            vizPoint.pose.orientation.w = 1.0
            vizPoint.scale.x = 0.5
            vizPoint.scale.y = 0.5
            vizPoint.scale.z = 0.5
            vizPoint.color.r = 1.0
            vizPoint.color.a = 1.0
            self.markerPub.publish(vizPoint)

       

def main(args=None):
    rclpy.init(args=args)


    simple_pure_pursuit_node = SimplePurePursuitNode()
    while rclpy.ok():
        rclpy.spin(simple_pure_pursuit_node)
    simple_pure_pursuit_node.destroy_node()
    rclpy.shutdown()
# if __name__ == "__main__":
#     try:
#         controller = SimplePurePursuitNode()
#         while rclpy.ok():
#             controller.loop()

#     except rospy.ROSInterruptException:
#         pass
