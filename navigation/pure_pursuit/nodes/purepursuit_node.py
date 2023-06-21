"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pure_pursuit import (
    WayPoints,
    State,
    PidController,
    purepursuitSteercontrol,
    Position,
    plot,
)

from nav_msgs.msg import Path, Odometry


PLOTTING = rospy.get_param("/plotting")
PLOTNAME = rospy.get_param("/plotName")
TARGETSPEED = rospy.get_param("/targetSpeed")


def main() -> None:
    """
    Main function for pure pursuit vehicle control node, subscribes to
    state and waypoints and publishes control actions, also plots the
    path and vehicle trajectory if plotting is set to true
    """
    rospy.init_node("purepursuit_controller", anonymous=True)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDriveStamped, queue_size=10)
    waypoints = WayPoints()

    position = Position(0.0, 0.0)
    state = State(position, 0.0)
    rospy.Subscriber("/state", Odometry, callback=state.update)
    rospy.Subscriber("/pathplanning/waypoints", Path, callback=waypoints.add)
    rospy.wait_for_message("/pathplanning/waypoints", Path)

    controlAction = AckermannDriveStamped()
    pidController = PidController()
    rate = rospy.Rate(rospy.get_param("/rate"))
    targetInd = 0
    prevError = 0.0
    while not rospy.is_shutdown():

        delta, targetInd = purepursuitSteercontrol(state, waypoints, targetInd)

        acc, error = pidController.proportionalControl(TARGETSPEED, state.currentSpeed, prevError)
        prevError = error

        controlAction.drive.acceleration = acc
        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = TARGETSPEED

        controlActionPub.publish(controlAction)
        if PLOTTING:

            plot(waypoints, state, PLOTNAME, targetInd)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
