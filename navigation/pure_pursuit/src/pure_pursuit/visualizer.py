"""
visualizing the waypoints & the current state and target point of the vehicle
using matplotlib library
"""

import matplotlib.pyplot as plt
from .pure_pursuit import WayPoints, State


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
