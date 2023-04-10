# pylint: disable=all
# mypy: ignore-errors
from pure_pursuit import State, WayPoints, Position
import numpy as np

position = Position(0, 0)
state = State(position, 0)
state.rearX = 0.0
state.rearY = 0.7
waypoints = WayPoints()
waypoints.xList = [1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
waypoints.yList = [1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9]
waypoints.oldNearestPointIndex = 0

for i in range(20):  #

    if waypoints.firstLoop is False:
        # search nearest point index
        distanceX = [state.rearX - icx for icx in waypoints.xList]
        distanceY = [state.rearY - icy for icy in waypoints.yList]
        distance = np.hypot(distanceX, distanceY)

        print("distance : " + str(distance))
        print("loop:", i)

        if len(distance) != 0:
            ind: int = int(np.argmin(distance))

            print("first index: " + str(ind))

            waypoints.oldNearestPointIndex = ind
            waypoints.firstLoop = True
        else:
            ind = 0

            print("first index: " + str(ind))

            waypoints.firstLoop = True

        # self.assert_(ind == 0)
    else:
        ind = waypoints.oldNearestPointIndex
        state.rearX = state.rearX + 0.3
        state.rearY = state.rearY - 0.2

        print("\n")

        print("loop:", i, ", index: " + str(ind))
        print("rear x: " + str(state.rearX), "rear y: " + str(state.rearY))

        distanceThisIndex = float(state.calcDistance(waypoints.xList[ind], waypoints.yList[ind]))
        # el state mmkn tkon et8ayart fel fatra ely elloop bt7sl feha fa hal btsm3 hena wla la ? w hal elsah enha tsm3 wla la ?
        # a3taked lazm tsama3 3shan eldistanceThisIndex tb2a as8ar men eldistanceNextIndex ya2ema hatefdal gowa elloop

        print("distance this index: ", distanceThisIndex)
        # print("calc distance output: ",state.calcDistance(waypoints.xList[ind], waypoints.yList[ind]))

        counter = 0
        while ind < len(waypoints.xList) - 1:
            distanceNextIndex = state.calcDistance(
                waypoints.xList[ind + 1], waypoints.yList[ind + 1]
            )
            counter = counter + 1

            print("while loop counter: ", counter)
            # print("calc distance output: ",state.calcDistance(waypoints.xList[ind + 1], waypoints.yList[ind + 1])) #htfail lw elwaypoints 5elso
            print("distance next index: " + str(distanceNextIndex))
            # print("for loop counter:", i)

            if (
                distanceThisIndex < distanceNextIndex
            ):  # lazm at2aked en el etnen waypoints msh nfs eldistance benhom w ben eldistance ya2ema hyfdal gowa elloop 3la tol msh hybreak
                print("breaked")
                break
            ind = ind + 1 if (ind + 1) < len(waypoints.xList) else ind
            distanceThisIndex = distanceNextIndex
            print("index value: " + str(ind))
        waypoints.oldNearestPointIndex = ind
        print("index before going to next loop: " + str(ind))
