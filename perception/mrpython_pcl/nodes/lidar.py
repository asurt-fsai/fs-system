#!/usr/bin/python3
"""
Main ros node for the lidar pipeline used to detect cones
"""
import rclpy
from rclpy.node import Node
from mrpython_pcl.ros.Builders import Builder
from tf_helper.StatusPublisher import StatusPublisher

class LidarSystem(Node):
    def __init__(self):
        super().__init__("mr_lidar")
        self.create_timer(0.1, self.timer_callback)
        self.status = StatusPublisher("/status/lidar")
        self.builder = Builder()
        self.status.starting()
        self.lidar = self.builder.buildPipeline()
        self.status.ready()

    def timer_callback(self):
        
        try:
            out = self.lidar.run()
        except Exception as exp:  # pylint: disable=broad-except
            self.get_logger().warn("Lidar Pipeline failed: " + str(exp))
            return
        if out is None:
            return
        self.status.running()
        


def main(args = None) -> None:
    """
    Main Loop
    """
    rclpy.init(args = args)
    node = LidarSystem()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
