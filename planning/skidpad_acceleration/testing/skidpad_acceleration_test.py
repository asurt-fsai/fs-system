#!/usr/bin/python
# pylint: disable=all
# mypy: ignore-errors
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

global CAR_POSITION_PUB


def Update_car_position(points) -> None:
    """
    This method updates the car's position and publishes it
    """
    global CAR_POSITION_PUB
    CAR_POSITION_PUB.publish(points.poses[0])


def Car_node() -> None:
    """
    This method initializes the car node and subscribes to the acceleration topic
    and publishes the car's position
    """
    global CAR_POSITION_PUB
    rospy.init_node("acceleration_node", anonymous=True)

    rospy.Subscriber("acceleration_topic", Path, Update_car_position)
    CAR_POSITION_PUB = rospy.Publisher("/car_position", PoseStamped, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    try:
        Car_node()
    except rospy.ROSInterruptException:
        pass
