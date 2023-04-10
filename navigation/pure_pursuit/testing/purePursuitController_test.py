# pylint: disable=all
# mypy: ignore-errors
from pure_pursuit import State, WayPoints, Position
import math
from geometry_msgs.msg import Pose

BASELENGTH = 2.9


def purepursuitController_test(state: State, trajectory: WayPoints, pind: int):
    print(
        "state X: {}, state Y: {}, state yaw: {}, state rearX: {}, state rearY: {}, currentspeed: {}".format(
            state.position.x,
            state.position.y,
            state.yaw,
            round(state.rearX, 4),
            state.rearY,
            state.currentSpeed,
        )
    )
    ind, lookAhead = trajectory.searchTargetIndex(state)
    trajX: float = 0
    trajY: float = 0
    if pind >= ind:
        ind = pind
        print("first loop ind: ", ind)
    if ind < len(trajectory.xList):
        trajX = trajectory.xList[ind]
        trajY = trajectory.yList[ind]
        print("ind: ", ind)

    else:  # toward goal
        trajX = trajectory.xList[-1]
        trajY = trajectory.yList[-1]
        ind = len(trajectory.xList) - 1
        print("last loop ind: ", ind)
    alpha: float = math.atan2(trajY - state.rearY, trajX - state.rearX) - state.yaw
    print("alpha: ", alpha)
    print("trajX: ", trajX, " trajY: ", trajY)
    # alpha  = -math.pi/2
    lookAhead = 0.83 * BASELENGTH
    delta: float = math.atan2(2.0 * BASELENGTH * math.sin(alpha) / lookAhead, 1.0)
    deltaDegree = math.degrees(delta)
    print("lookAhead: ", lookAhead)
    print("delta: ", delta)
    print("deltaDegree: ", deltaDegree)
    print("\n")
    return delta, ind


trajectory = WayPoints()
trajectory.xList = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19]
trajectory.yList = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19]
firstpose = Position(0, 0)
state = State(firstpose, 0, 0)
newpose = Pose()
for i in range(10):
    purepursuitController_test(state, trajectory, i)

    newpose.position.x += 1
    newpose.position.y += 1

    state.update(newpose)
    # print(newpose.position.x, newpose.position.y)
