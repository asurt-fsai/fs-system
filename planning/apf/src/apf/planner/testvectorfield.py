#!/usr/bin/env python3
"""
APF module
"""
from typing import Tuple, List, Any
from matplotlib.patches import Circle
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import rospy
from asurt_msgs.msg import LandmarkArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class APF:  # pylint: disable=too-many-instance-attributes
    # doc string in numpy style
    """
    An implementation of the Attractive Potential Field (APF) algorithm.

    Parameters
    ----------
    start : Tuple[float, float]
        The starting point of the robot.
    goal : Tuple[float, float]
        The goal point of the robot.
    obstacles : list
        A list of obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    kAttractive : float
        The attractive force constant.
    kRepulsive : float
        The repulsive force constant.
    pNode : float
        The node constant.
    stepSize : float
        The step size.
    maxIterations : int
        The maximum number of iterations.
    goalThreshold : float
        The threshold distance from the goal.
    obsYellow : list
        A list of yellow obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    obsBlue : list
        A list of blue obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    plot : bool
        True if the plot should be shown, false otherwise.

    Attributes
    ----------
    start : npt.NDArray[np.float64]
        The starting point of the robot.
    currentPosition : npt.NDArray[np.float64]
        The current position of the robot.
    goal : npt.NDArray[np.float64]
        The goal point of the robot.
    obstacles : npt.NDArray[np.float64]
        A list of obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    kAttractive : float
        The attractive force constant.
    kRepulsive : float
        The repulsive force constant.
    pNode : float
        The node constant.
    stepSize : float
        The step size.
    maxIterations : int
        The maximum number of iterations.
    iterations : int
        The number of iterations.
    goalThreshold : float
        The threshold distance from the goal.
    path : npt.NDArray[np.float64]
        The path found by the algorithm.
    isPathPlanSuccess : bool
        True if the path was found, false otherwise.
    deltaT : float
        The time step.
    """

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacles: npt.NDArray[np.float64],
        kAttractive: float,
        kRepulsive: float,
        pNode: float,
        stepSize: float,
        maxIterations: int,
        goalThreshold: float,
        obsYellow: npt.NDArray[np.float64],
        obsBlue: npt.NDArray[np.float64],
        plot: bool,
    ):
        self.start = np.array([start[0], start[1]])
        self.currentPosition = np.array([float(start[0]), float(start[1])])
        self.goal = np.array([goal[0], goal[1]])
        self.obstacles = [np.array([obstacle[0], obstacle[1]]) for obstacle in obstacles]
        self.kAttractive = kAttractive
        self.kRepulsive = kRepulsive
        self.pNode = pNode
        self.stepSize = stepSize
        self.maxIterations = maxIterations
        self.iterations = 0
        self.goalThreshold = goalThreshold
        self.path: npt.NDArray[np.float64] = self.start
        self.isPathPlanSuccess = False
        self.deltaT = 0.01
        self.unduplicatedObstacles = np.copy(self.obstacles)
        self.flag = False
        self.obsYellow: npt.NDArray[np.float64] = np.array(
            [np.array([OB[0], OB[1]]) for OB in obsYellow]
        )
        self.obsBlue: npt.NDArray[np.float64] = np.array(
            [np.array([obstacle[0], obstacle[1]]) for obstacle in obsBlue]
        )
        self.plot = plot
        self.oneSide = False
        self.passedCones = False

    def length(self, force: npt.NDArray[np.float64]) -> float:
        """
        Computes the length of a force vector.

        Parameters
        ----------
        force : npt.NDArray[np.float64]
            The force vector.

        Returns
        -------
        length : float
            The length of the force vector.
        """
        value: float = np.linalg.norm(force, axis=0)
        return value

    def direction(self, force: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Computes the direction of a force vector.

        Parameters
        ----------
        force : npt.NDArray[np.float64]
            The force vector.

        Returns
        -------
        direction : npt.NDArray[np.float64]
            The direction of the force vector.
        """

        if self.length(force) == 0:
            return np.array([0.0, 0.0])
        return np.array([force[0], force[1]]) * (1 / self.length(force))

    def nearest(
        self, obstacles: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Find the nearest obstacle.

        Parameters
        ----------
        obstacles : npt.NDArray[np.float64]
            A list of obstacles in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].

        Returns
        -------
        Tuple[Tuple[float, float], Tuple[float, float]]
        nearestObstacle: Tuple[float, float]
            The nearest obstacle.
        nearestToObstacle: Tuple[float, float]
            The vector from the current position to the nearest obstacle.
        """
        nearestObstacle = self.obstacles[0]
        nearestToObstacle = np.full((2,), np.inf)
        for obstacle in obstacles:
            if self.length((obstacle) - self.currentPosition) < self.length(nearestToObstacle):
                nearestObstacle = obstacle
                nearestToObstacle = nearestObstacle - self.currentPosition

        return nearestObstacle, nearestToObstacle

    def newAttractive(self) -> Any:
        """
        Computes the attractive force vector.

        Returns
        -------
        attractiveToGoal : npt.NDArray[np.float64]
            The attractive force vector to the goal.
        """
        attractiveToObstacle = np.array([0.0, 0.0])
        blue = False
        yellow = False

        if self.flag is False and (np.any(self.obsBlue) and np.any(self.obsYellow)):
            # two sides are visible or if only blue is seen from the other track
            nearestBlue, nearestToBlueObstacle = self.nearest(self.obsBlue)

            nearestYellow, nearestToYellowObstacle = self.nearest(self.obsYellow)
            # print(
            #     "self.length(nearestBlue - nearestYellow)", self.length(nearestBlue - nearestYellow)
            # )
            # print("yellow.x,yellow.y", nearestYellow[0], nearestYellow[1])

            if (
                self.length(nearestBlue - nearestYellow) <= 4.7
                and self.currentPosition[0] < ((nearestBlue + nearestYellow) / 2)[0]
            ):
                print("two sides")
                # print(self.length(nearestBlue - nearestYellow))

                return (
                    ((nearestBlue + nearestYellow) / 2) - self.currentPosition
                ) * self.kAttractive
            else:
                self.flag = True
                if (self.length(nearestToBlueObstacle) < self.length(nearestToYellowObstacle)) and (
                    self.length(nearestBlue - nearestYellow) > 4.7
                ):
                    # this means that yellow/blue are from different tracks will be deleted
                    # will delete yellow
                    print("blue")
                    # print(self.length(nearestBlue - nearestYellow))
                    blue = True

                elif (
                    self.length(nearestToBlueObstacle) > self.length(nearestToYellowObstacle)
                ) and (self.length(nearestBlue - nearestYellow) > 4.7):
                    yellow = True

                if self.plot is True:
                    print("self.flaggg= falseeee")
                self.flag = True

        oneSide = bool(np.any(self.obsBlue)) is False or bool(np.any(self.obsYellow)) is False
        # oneSide = False

        if self.flag is True or oneSide:
            if self.flag and not oneSide:
                xCoordsBlue = np.array([obsBlue[0] for obsBlue in self.obsBlue])
                xCoordsYellow = np.array([obsYellow[0] for obsYellow in self.obsYellow])
                if np.all(self.currentPosition[0] > 1.2 * xCoordsBlue) or np.all(
                    self.currentPosition[0] > 1.2 * xCoordsYellow
                ):
                    print("passed all blue/yellow cones")
                    # passed all blue/yellow cones ,only yellow/blue cones left
                    # equivalent to one side
                    oneSide = True
                    self.passedCones = True

            if oneSide:
                self.kRepulsive *= 10
                self.oneSide = True
            # only one side visible
            if blue is True:
                print("deleting yellow")
                # then delete all yellow obstacles from obstacles
                for eachObstacle in self.obsYellow:
                    # delete eachObstacle from self.obstacles
                    self.unduplicatedObstacles = self.unduplicatedObstacles[
                        (self.unduplicatedObstacles != eachObstacle).all(axis=1)
                    ]
                self.obsYellow = np.array([])

            elif yellow is True:
                # then delete all blue obstacles from obstacles
                print("deleting blue")
                for eachObstacle in self.obsBlue:
                    # delete obfrom self.obstacles
                    self.unduplicatedObstacles = self.unduplicatedObstacles[
                        (self.unduplicatedObstacles != eachObstacle).all(axis=1)
                    ]
                self.obsBlue = np.array([])

            for obstacle in self.unduplicatedObstacles:
                attractiveToObstacle += (obstacle - self.currentPosition) * self.kAttractive

                if obstacle[0] < (self.currentPosition[0]):
                    # this means current position is past the nearest obstacle
                    self.unduplicatedObstacles = self.unduplicatedObstacles[
                        (self.unduplicatedObstacles != obstacle).all(axis=1)
                    ]

                    if self.unduplicatedObstacles.size == 2:
                        self.isPathPlanSuccess = True
                        print("self.isPathPlanSuccess", self.isPathPlanSuccess)
                        break

            return attractiveToObstacle

    def repulsion(self) -> npt.NDArray[np.float64]:
        """
        Computes the repulsive force vector.

        Returns
        -------
        rep : npt.NDArray[np.float64]
            The repulsive force vector.
        """
        rep = np.zeros(2, dtype=np.float128)
        total = np.zeros(2, dtype=np.float128)

        for obstacle in self.obstacles:
            obsToRob = self.currentPosition - obstacle

            robToObs = obstacle - self.currentPosition

            if self.length(obsToRob) > self.pNode:
                pass

            else:
                rep = (
                    np.array(
                        [
                            self.direction(obsToRob)[0],
                            self.direction(obsToRob)[1],
                        ]
                    )
                    * self.kRepulsive
                    * (1.0 / self.length(obsToRob) - 1.0 / self.pNode)
                    / (self.length(obsToRob) ** 2)
                )

                rep2 = (
                    np.array(
                        [
                            self.direction(robToObs)[0],
                            self.direction(robToObs)[1],
                        ]
                    )
                    * self.kRepulsive
                    * ((1.0 / self.length(robToObs) - 1.0 / self.pNode) ** 2)
                    * self.length(obsToRob)
                )

                total = total + rep2 + rep

        return total

    def pathPlanPlot(self) -> None:
        """
        Plots the path plan.
        """
        resultantForceVector = np.zeros(2, dtype=np.float128)
        if self.plot is True:
            fig4 = plt.figure(4)
            start = (0, 0)
            subplot = fig4.add_subplot(111)
            subplot.set_xlabel("X-distance: m")
            subplot.set_ylabel("Y-distance: m")
            subplot.plot(start[0], start[1], "*r")

            if np.any(self.obsYellow):
                for obstacle in self.obsYellow:
                    circle = Circle(xy=(obstacle[0], obstacle[1]), radius=self.pNode, alpha=0.3)
                    subplot.add_patch(circle)
                    # change the colour of patch to be yellow
                    circle.set_facecolor("yellow")
                    subplot.plot(obstacle[0], obstacle[1], "xk")

            if np.any(self.obsBlue):
                for obstacle in self.obsBlue:
                    circle = Circle(xy=(obstacle[0], obstacle[1]), radius=self.pNode, alpha=0.3)
                    subplot.add_patch(circle)
                    # change the colour of patch to be blue
                    circle.set_facecolor("blue")
                    subplot.plot(obstacle[0], obstacle[1], "xk")

        while self.iterations < self.maxIterations:
            if self.oneSide or (self.passedCones and self.oneSide):
                print("oneside")
                self.pNode = 1.8

                attractive = self.newAttractive()
                attractiveNorm = np.linalg.norm(attractive)  # compute the norm of the vector
                if attractiveNorm > 0:
                    attractiveUnit = attractive / attractiveNorm  # normalize the vector
                else:
                    attractiveUnit = np.zeros(2)  # handle the case where the vector is zero

                resultantForceVector = attractiveUnit + self.repulsion()

                # resultantForceVector = 0.000000000000000001 + self.repulsion() ** 3

            else:
                resultantForceVector = self.newAttractive() + self.repulsion()

            self.currentPosition += (
                np.array(
                    [
                        self.direction(resultantForceVector)[0],
                        (self.direction(resultantForceVector)[1]),
                    ]
                )
                * self.stepSize
            )
            self.iterations += 1
            # extend / append

            self.path = np.vstack(
                (
                    self.path,
                    np.array([self.currentPosition[0], self.currentPosition[1]]),
                )
            )

            if self.plot is True:
                subplot.plot(self.currentPosition[0], self.currentPosition[1], ".b")
                fig4.canvas.draw()
        plt.pause(self.deltaT)
        plt.clf()
        self.isPathPlanSuccess = True


def marker_callback(msg):
    global YELLOW_CONES, BLUE_CONES, ALL_CONES
    # print(len(msg.markers))
    YELLOW_CONES = []
    BLUE_CONES = []
    ALL_CONES = []
    for marker in msg.markers:
        # print("marker",marker.color)
        # print(marker.pose)
        r, g, b = marker.color.r, marker.color.g, marker.color.b
        # print(r,g,b)
        x, y = marker.pose.position.x, marker.pose.position.y
        if r == 1 and g == 1 and b == 0:
            YELLOW_CONES.append([x, y])
            ALL_CONES.append([x, y])
        elif r == 0 and b == 1:
            BLUE_CONES.append([x, y])
            ALL_CONES.append([x, y])
    # time.sleep(0.1)


BLUE_CONES = []
ALL_CONES = []
YELLOW_CONES = []
rospy.init_node("object_list_to_array")

object_list_sub = rospy.Subscriber(
    "/carmaker/ObjectList", MarkerArray, marker_callback, queue_size=10
)
waypointsPub = rospy.Publisher("/pathplanning/waypoints", Path, queue_size=10)


# def perceptionCallback(landmarkArray: npt.NDArray[np.float64]) -> None:
#     """
#     This callback function is called whenever a new message of type LandmarkArray is
#     received by the subscriber.

#     Parameters
#     ----------
#     LandmarkArray : LandmarkArray
#         The message received by the subscriber.

#     Returns
#     -------
#     None.

#     """
#     global YELLOW_CONES, BLUE_CONES, ALL_CONES
#     YELLOW_CONES = []
#     BLUE_CONES = []
#     ALL_CONES = []
#     for landmark in landmarkArray.landmarks:
#         if landmark.type == 0:
#             BLUE_CONES.append(np.array([landmark.position.x, landmark.position.y]))
#             ALL_CONES.append(np.array([landmark.position.x, landmark.position.y]))
#         elif landmark.type == 1:
#             YELLOW_CONES.append(np.array([landmark.position.x, landmark.position.y]))
#             ALL_CONES.append(np.array([landmark.position.x, landmark.position.y]))
#         elif landmark.type == 2:
#             YELLOW_CONES.append(np.array([landmark.position.x, landmark.position.y]))
#             ALL_CONES.append(np.array([landmark.position.x, landmark.position.y]))


# YELLOW_CONES: npt.NDArray[np.float64] = []
# BLUE_CONES: npt.NDArray[np.float64] = []
# ALL_CONES: npt.NDArray[np.float64] = []

# rospy.init_node("apf_test")
# # for rosbag
# rospy.Subscriber("/perception/smornn/detected", LandmarkArray, perceptionCallback, queue_size=10)

# rospy.Subscriber("/cones_map", LandmarkArray, perceptionCallback, queue_size=10)


def numpyToPath(pathArray: npt.NDArray[np.float64]) -> Path:
    """
    Converts a numpy array to a path message
    and transforms the coordinates to the world frame
    Parameters
    ----------
    pathArray: np.ndarray, shape=(n, 2)
        Numpy array with x and y coordinates of the path
    Returns
    -------
    path: Path
        Path message
    """
    achievedPath = Path()
    for i in range(pathArray.shape[0]):
        pose = PoseStamped()
        pose.pose.position.x = pathArray[i][0]
        pose.pose.position.y = pathArray[i][1]
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        achievedPath.poses.append(pose)
    return achievedPath


if __name__ == "__main__":
    startTest, goalTest = (0, 0), (0, 0)
    while not rospy.is_shutdown():
        if len(ALL_CONES) > 0:
            apfTest = APF(
                (0, 0),
                goalTest,
                ALL_CONES,
                3.5,
                30,
                1.3,
                0.2,
                25,
                0.2,
                YELLOW_CONES,
                BLUE_CONES,
                True,
            )
            apfTest.pathPlanPlot()

            path = Path()
            npath = np.array(apfTest.path)
            path = numpyToPath(npath)
            waypointsPub.publish(path)

        # print(path)
