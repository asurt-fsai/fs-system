"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from simple_pure_pursuit import WayPoints
from nav_msgs.msg import Path

TARGETSPEED = rospy.get_param("/speed/target", 10)


def main() -> None:
    """
    Main function for simple pure pursuit vehicle control node, subscribes to
    waypoints and publishes control actions
    """
    rospy.init_node("simple_pp_controller", anonymous=True)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDriveStamped, queue_size=10)
    waypoints = WayPoints()

    rospy.Subscriber("/pathplanning/waypoints", Path, callback=waypoints.add)

    controlAction = AckermannDriveStamped()

    rate = rospy.Rate(rospy.get_param("/rate", 10))
    targetInd = 0

    while not rospy.is_shutdown():

        delta, targetInd = waypoints.purepursuitSteercontrol(targetInd)

        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = TARGETSPEED

        controlActionPub.publish(controlAction)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
