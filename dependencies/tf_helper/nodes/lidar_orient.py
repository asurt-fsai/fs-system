#!/usr/bin/python3
"""
Ros node to subscribe to velodyne_points and republish it in the fixed horizontal orientation
The lidar is placed at an angle so this node transforms the points
by the inverse of that angle to not affect slam
"""
import rclpy
from rclpy.node import Node

from tf_helper_ros2.TFHelper import TFHelper
from tf_helper_ros2.StatusPublisher import StatusPublisher
from sensor_msgs.msg import PointCloud2


class VelodyneTransformer:  # pylint: disable=too-few-public-methods
    """
    Class to transform the velodyne points to the fixed frame of reference (horizontal)

    Parameters
    ----------
    velodyneTopic : str
        The topic on which the velodyne points are published
    velodyneFixedTopic : str
        The topic on which the transformed velodyne points are published
    velodyneFixedFrame : str
        The fixed frame of reference to which the velodyne points are transformed
    """

    def __init__(self, velodyneTopic: str, velodyneFixedTopic: str, velodyneFixedFrame: str):
        super().__init__('lidar_orient')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('VelodyneTopic', ''),
                                    ('VelodyneFixedTopic', ''),
                                    ('VelodyneFixedFrame', '')
                                ])
        VelodyneTopic = self.get_parameter('VelodyneTopic').get_parameter_value().string_value
        VelodyneFixedTopic = self.get_parameter('VelodyneFixedTopic').get_parameter_value().string_value
        VelodyneFixedFrame = self.get_parameter('VelodyneFixedFrame').get_parameter_value().string_value

        self.status = StatusPublisher(self, "/status/lidar_orient")
        self.status.starting()

        self.helper = TFHelper(self)
        self.VelodyneFixedFrame = VelodyneFixedFrame
        
        self.fixed_pc_pub = self.create_publisher(PointCloud2, VelodyneFixedTopic, 10)
        self.subscription = self.create_subscription(PointCloud2, VelodyneTopic, self.pc_callback, 10)
        
        self.status.ready()

    def pcCallback(self, pointcloud: PointCloud2) -> None:
        """
        Callback function for the Velodyne points.
        Publishes the transformed Velodyne points.

        Parameters
        ----------
        pointcloud : PointCloud2
            The Velodyne points.
        """
        fixedPc = self.helper.transformMsg(pointcloud, self.velodyneFixedFrame)  # Adjusted for ROS 2
        self.fixedPcPub.publish(fixedPc)
        self.status.running()


def main(args=None):
    rclpy.init(args=args)
    # In ROS 2, consider using the parameter server or declare_parameters to handle parameters.
    transformer = VelodyneTransformer()

    try:
        rclpy.spin(transformer)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()