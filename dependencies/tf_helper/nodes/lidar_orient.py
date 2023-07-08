#!/usr/bin/python3
"""
Ros node to subscribe to velodyne_points and republish it in the fixed horizontal orientation
The lidar is placed at an angle so this node transforms the points
by the inverse of that angle to not affect slam
"""
import rospy

from tf_helper.TFHelper import TFHelper
from tf_helper.StatusPublisher import StatusPublisher
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
        self.status = StatusPublisher("/status/lidar_orient")
        self.status.starting()

        self.helper = TFHelper("lidar_orient")
        self.velodyneFixedFrame = velodyneFixedFrame

        self.fixedPcPub = rospy.Publisher(velodyneFixedTopic, PointCloud2, queue_size=10)

        rospy.Subscriber(velodyneTopic, PointCloud2, self.pcCallback)
        self.status.ready()

    def pcCallback(self, pointcloud: PointCloud2) -> None:
        """
        Callback function for the velodyne points
        Publishes the transformed velodyne points

        Parameters
        ----------
        pointcloud : PointCloud2
            The velodyne points
        """
        fixedPc = self.helper.transformMsg(pointcloud, self.velodyneFixedFrame)
        self.fixedPcPub.publish(fixedPc)
        self.status.running()


def main() -> None:
    """
    Main Loop
    """
    rospy.init_node("lidar_orient")

    velodyneTopic = rospy.get_param("/tf_helper/velodyne_topic")
    velodyneFixedTopic = rospy.get_param("/tf_helper/velodyne_fixed_topic")
    velodyneFixedFrame = rospy.get_param("/tf_helper/velodyne_fixed_frame")

    VelodyneTransformer(velodyneTopic, velodyneFixedTopic, velodyneFixedFrame)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
