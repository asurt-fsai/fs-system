#!/usr/bin/python3
# pylint: disable=all
# mypy: ignore-errors
"""
ROS node for testing the reception of skidpad waypoints.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class CarNode:
    """
    ROS node for testing the reception of skidpad waypoints.
    """

    def __init__(self) -> None:
        self.car_position_pub = None

    def update_car_position(self, points) -> None:
        """
        Update the car's position based on the provided points.

        Parameters:
        ------------
        points : Path
            Path object containing the car's updated position.

        Returns:
        -----------
        None
        """
        self.car_position_pub.publish(points.poses[0])

    def run(self) -> None:
        """
        Initialize the car node and set up the necessary publishers and subscribers.

        Parameters:
        ------------
        None

        Returns:
        -----------
        None
        """
        rospy.init_node("car_node", anonymous=True)

        rospy.Subscriber("/waypoints_topic", Path, self.update_car_position)
        self.car_position_pub = rospy.Publisher("/car_position", PoseStamped, queue_size=10)
        rospy.spin()


if __name__ == "__main__":
    try:
        node = CarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
