# pylint: disable=all
# mypy: ignore-errors
import rospy
import math

from asurt_msgs.msg import NodeStatus
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
from pure_pursuit import (
    WayPoints,
    State,
    purepursuitSteercontrol,
    proportionalControl,
    targetSpeedCalc,
    Position,
    previousError,
)
from nav_msgs.msg import Path
import matplotlib.pyplot as plt


message = NodeStatus()
message.status = "ready"
plotting = rospy.get_param("/plotting")


def main():
    """
    main function for pure pursuit vehicle control
    """
    rospy.init_node("purepursuit_controller", anonymous=True)

    # rospy.wait_for_message("/waypoints", Pose)
    # rospy.wait_for_message("/state", Pose)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDrive, queue_size=10)
    waypoints = WayPoints()
    waypoints.xList = [0.0]
    waypoints.yList = [0.0]
    position = Position(0.0, 0.0)
    # alpha = math.atan2(waypoints.yList[targetInd] - state.rearY, waypoints.xList[targetInd] - state.rearX)
    state = State(position, 0.0)
    rospy.Subscriber("/state", Pose, callback=state.update)
    rospy.Subscriber("/waypoints", Pose, callback=waypoints.add)

    controlAction = AckermannDrive()
    # message.status = #ready
    rate = rospy.Rate(10)
    targetInd = 0

    while not rospy.is_shutdown() and message == "ready":  # and using_pure_pursuit = true
        # wait for atleast 1 waypoint

        # targetInd, _ = waypoints.searchTargetIndex(state)
        # ind , message = waypoints.searchTargetIndex(state)
        delta, targetInd, message = purepursuitSteercontrol(
            state, waypoints, targetInd
        )  # lateral controller
        targetSpeed = targetSpeedCalc(delta)
        # rospy.loginfo("test")
        prevError = previousError(targetSpeed, state.currentSpeed)
        acc = proportionalControl(
            targetSpeed, state.currentSpeed, prevError
        )  # longitudinal controller
        deltaDegree = math.degrees(delta)
        controlAction.acceleration = acc  # proportionalControl(targetSpeed, state.currentSpeed)
        controlAction.steering_angle = delta
        controlAction.jerk = targetInd
        controlAction.speed = deltaDegree
        controlAction.steering_angle_velocity = waypoints.yList[targetInd]
        rospy.loginfo(message)
        # clearance = state.calcDistance(waypoints.X_coordinates[-1], waypoints.Y_coordinates[-1])
        # if clearance <= 1.4:
        #     # goal_reached = True
        #     print("Goal Reached")
        #     controlAction.acceleration = 0.0
        #     controlAction.steering_angle = 0.0
        testname = "fixing indixing problem/adaptive lookahead"
        controlActionPub.publish(controlAction)
        if plotting:

            plot(waypoints, state, testname, targetInd)
        # message.status = #running

        rate.sleep()

    # assert lastIndex >= targetInd, "Cannot reach goal"


def plot(waypoints, state, name, target_ind):
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
    plt.plot(waypoints.xList[target_ind], waypoints.yList[target_ind], "xg", label="target")
    plt.axis("equal")
    plt.title(name)
    # plt.legend()
    plt.pause(0.001)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
