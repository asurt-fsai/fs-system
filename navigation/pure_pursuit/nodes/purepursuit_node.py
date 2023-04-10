"""
Initilization Pure Pursuit node for vehicle control
"""
import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, proportionalControl, Position
import matplotlib.pyplot as plt


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

    controlActionPub = rospy.Publisher("/control_actions", AckermannDrive, queue_size=10)
    waypoints = WayPoints()
    waypoints.xList = [0.0]
    waypoints.yList = [0.0]
    position = Position(0.0, 0.0)
    state = State(position, 0.0)
    rospy.Subscriber("/state", Pose, callback=state.update)
    rospy.Subscriber("/waypoints", Pose, callback=waypoints.add)

    controlAction = AckermannDrive()

    rate = rospy.Rate(10)
    targetInd = 0
    prevError = 0
    while not rospy.is_shutdown():

        delta, targetInd = purepursuitSteercontrol(state, waypoints, targetInd)

        acc, error = proportionalControl(TARGETSPEED, state.currentSpeed, prevError)
        # longitudinal controller
        prevError = error
        deltaDegree = math.degrees(delta)
        controlAction.acceleration = acc  # proportionalControl(targetSpeed, state.currentSpeed)
        controlAction.steering_angle = delta
        controlAction.jerk = targetInd
        controlAction.speed = deltaDegree
        controlAction.steering_angle_velocity = waypoints.yList[targetInd]

        controlActionPub.publish(controlAction)
        if PLOTTING:

            plot(waypoints, state, PLOTNAME, targetInd)

        rate.sleep()

    # assert lastIndex >= targetInd, "Cannot reach goal"


def plot(waypoints: WayPoints, state: State, name: str, targetInd: int) -> None:
    """
    plot the waypoints and the current state of the vehicle

    Args:
        waypoints (WayPoints): waypoints to follow

        state (State): current state of the vehicle

        name (str): name of the plot
    """
    plt.plot(waypoints.xList, waypoints.yList, "-r", label="course")
    # plt.plot(waypoints.xList[targetInd], waypoints.yList[targetInd], "xg", label="target")
    plt.plot(state.position.x, state.position.y, "ob", label="state")
    plt.grid(True)
    plt.plot(waypoints.xList[targetInd], waypoints.yList[targetInd], "xg", label="target")
    plt.axis("equal")
    plt.title(name)
    # plt.legend()
    plt.pause(0.001)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
