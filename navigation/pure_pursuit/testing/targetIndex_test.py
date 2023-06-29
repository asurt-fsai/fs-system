# pylint: disable=all
# mypy: ignore-errors
# from pure_pursuit import State, WayPoints, Position
import numpy as np
from pure_pursuit import (
    WayPoints,
    State,
    purepursuitSteercontrol,
    Position,
    plot,
)
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

position = Position(0, 0)
state = State(position, 0)
# state.rearX = 0.0
# state.rearY = 0.7
waypoints = WayPoints()
waypoints.waypoints = Path()
# waypoints.waypoints.poses.append()
pose = PoseStamped()
# pose.pose.position.x = 1
# pose.pose.position.y = 0
pose1 = PoseStamped()
pose2 = PoseStamped()
pose3 = PoseStamped()
pose4 = PoseStamped()
pose5 = PoseStamped()
pose6 = PoseStamped()
pose7 = PoseStamped()
pose8 = PoseStamped()
pose9 = PoseStamped()
pose1.pose.position.x = 1
pose2.pose.position.x = 2
pose3.pose.position.x = 3
pose4.pose.position.x = 4
pose5.pose.position.x = 10
pose6.pose.position.x = 11
pose7.pose.position.x = 12
pose8.pose.position.x = 13
pose9.pose.position.x = 14

# lst = []
# for i in range(10):
#     # pose.pose.position.x = i
#     print(i)
#     # pose.pose.position.y = 0
#     # print(pose)
#     lst = lst + [pose]
#     print(lst)
waypoints.waypoints.poses.append(pose1)
waypoints.waypoints.poses.append(pose2)
waypoints.waypoints.poses.append(pose3)
waypoints.waypoints.poses.append(pose4)
waypoints.waypoints.poses.append(pose5)
waypoints.waypoints.poses.append(pose6)
waypoints.waypoints.poses.append(pose7)
waypoints.waypoints.poses.append(pose8)
waypoints.waypoints.poses.append(pose9)
# waypoints.waypoints.poses.append(pose1)
# waypoints.waypoints.poses.pose.append(pose)
# print(type(waypoints.waypoints.poses))

# print(waypoints.waypoints.poses)
# waypoints.xList = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# waypoints.yList = [0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0]
# waypoints.oldNearestPointIndex = 0
# print(waypoints.waypoints.poses)
print(waypoints.waypoints.poses)
print(waypoints.searchTargetIndex(state))
print(waypoints.xList)
# for i in range(20):  #

#     if waypoints.firstLoop is False:
#         # search nearest point index
#         distanceX = [state.rearX - icx for icx in waypoints.xList]
#         distanceY = [state.rearY - icy for icy in waypoints.yList]
#         distance = np.hypot(distanceX, distanceY)

#         print("distance : " + str(distance))
#         print("loop:", i)

#         if len(distance) != 0:
#             ind: int = int(np.argmin(distance))

#             print("first index: " + str(ind))

#             waypoints.oldNearestPointIndex = ind
#             waypoints.firstLoop = True
#         else:
#             ind = 0

#             print("first index: " + str(ind))

#             waypoints.firstLoop = True

#         # self.assert_(ind == 0)
#     else:
#         ind = waypoints.oldNearestPointIndex
#         state.rearX = state.rearX + 0.3
#         state.rearY = state.rearY - 0.2

#         print("\n")

#         print("loop:", i, ", index: " + str(ind))
#         print("rear x: " + str(state.rearX), "rear y: " + str(state.rearY))

#         distanceThisIndex = float(state.calcDistance(waypoints.xList[ind], waypoints.yList[ind]))
#         # el state mmkn tkon et8ayart fel fatra ely elloop bt7sl feha fa hal btsm3 hena wla la ? w hal elsah enha tsm3 wla la ?
#         # a3taked lazm tsama3 3shan eldistanceThisIndex tb2a as8ar men eldistanceNextIndex ya2ema hatefdal gowa elloop

#         print("distance this index: ", distanceThisIndex)
#         # print("calc distance output: ",state.calcDistance(waypoints.xList[ind], waypoints.yList[ind]))

#         counter = 0
#         while ind < len(waypoints.xList) - 1:
#             distanceNextIndex = state.calcDistance(
#                 waypoints.xList[ind + 1], waypoints.yList[ind + 1]
#             )
#             counter = counter + 1

#             print("while loop counter: ", counter)
#             # print("calc distance output: ",state.calcDistance(waypoints.xList[ind + 1], waypoints.yList[ind + 1])) #htfail lw elwaypoints 5elso
#             print("distance next index: " + str(distanceNextIndex))
#             # print("for loop counter:", i)

#             if (
#                 distanceThisIndex < distanceNextIndex
#             ):  # lazm at2aked en el etnen waypoints msh nfs eldistance benhom w ben eldistance ya2ema hyfdal gowa elloop 3la tol msh hybreak
#                 print("breaked")
#                 break
#             ind = ind + 1 if (ind + 1) < len(waypoints.xList) else ind
#             distanceThisIndex = distanceNextIndex
#             print("index value: " + str(ind))
#         waypoints.oldNearestPointIndex = ind
#         print("index before going to next loop: " + str(ind))
