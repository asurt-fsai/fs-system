"""
APF module
"""
from typing import Tuple, Any
from matplotlib.patches import Circle
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt


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
    repulsiveRadius : float
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
    repulsiveRadius : float
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
        repulsiveRadius: float,
        stepSize: float,
        maxIterations: int,
        goalThreshold: float,
        obsYellow: npt.NDArray[np.float64],
        obsBlue: npt.NDArray[np.float64],
        plot: bool,
    ):  # pylint: disable=too-many-arguments
        self.start = np.array([start[0], start[1]])
        self.currentPosition = np.array([float(start[0]), float(start[1])])
        self.goal = np.array([goal[0], goal[1]])
        self.obstacles = [np.array([obstacle[0], obstacle[1]]) for obstacle in obstacles]
        self.kAttractive = kAttractive
        self.kRepulsive = kRepulsive
        self.repulsiveRadius = repulsiveRadius
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
        self.extraYellow = np.array([])
        self.extraBlue = np.array([])

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

    def distanceToObstacles(self, obstacles: npt.NDArray[np.float64]):
        distances = [np.linalg.norm(cone - self.currentPosition) for cone in obstacles]
        sorted_cones = [cone for _, cone in sorted(zip(distances, obstacles))]
        return np.array(sorted_cones)

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

    def newAttractive(self) -> npt.NDArray[np.float128]:
        """
        Computes the attractive force vector.

        Returns
        -------
        attractiveToGoal : npt.NDArray[np.float64]
            The attractive force vector to the goal.
        """
        attractiveToObstacle = np.zeros(2, dtype=np.float128)
        nearestCone1, _ = self.nearest(self.unduplicatedObstacles)
        obstaclesWithoutNearestCone1 = self.unduplicatedObstacles[
            (self.unduplicatedObstacles != nearestCone1).all(axis=1)
        ]
        nearestCone2, _ = self.nearest(obstaclesWithoutNearestCone1)
        if (
            nearestCone1 in self.obsYellow
            and nearestCone2 in self.obsYellow
            or nearestCone1 in self.obsBlue
            and nearestCone2 in self.obsBlue
        ):
            obstaclesWithoutNearestCone2 = obstaclesWithoutNearestCone1[
                (obstaclesWithoutNearestCone1 != nearestCone2).all(axis=1)
            ]
            nearestCone3, _ = self.nearest(obstaclesWithoutNearestCone2)
            nearestCones = np.array([nearestCone1, nearestCone3])

        else:
            # if different colors
            nearestCones = np.array([nearestCone1, nearestCone2])

        for obstacle in nearestCones:
            if obstacle[0] < (self.currentPosition[0]):
                # this means current position is past the nearest obstacle
                self.unduplicatedObstacles = self.unduplicatedObstacles[
                    (self.unduplicatedObstacles != obstacle).all(axis=1)
                ]
                nearestCones = nearestCones[(nearestCones != obstacle).all(axis=1)]
            else:
                attractiveToObstacle += (obstacle - self.currentPosition) * self.kAttractive

            # if self.unduplicatedObstacles.size == 2:
            #     self.isPathPlanSuccess = True
            #     break

        return attractiveToObstacle

    def attractive2(self) -> npt.NDArray[np.float128]:
        attractiveToObstacle = np.zeros(2, dtype=np.float128)
        for obstacle in self.unduplicatedObstacles:
            if obstacle[0] < (self.currentPosition[0]):
                # this means current position is past the nearest obstacle
                self.unduplicatedObstacles = self.unduplicatedObstacles[
                    (self.unduplicatedObstacles != obstacle).all(axis=1)
                ]
            else:
                attractiveToObstacle += (obstacle - self.currentPosition) * self.kAttractive

            if self.unduplicatedObstacles.size == 2:
                self.isPathPlanSuccess = True
                break
        return attractiveToObstacle

    def repulsion(self) -> npt.NDArray[np.float128]:
        """
        Computes the repulsive force vector.

        Returns
        -------
        rep : npt.NDArray[np.float64]
            The repulsive force vector.
        """
        rep = np.zeros(2, dtype=np.float128)
        total = np.zeros(2, dtype=np.float128)
        for obstacle in self.extraYellow:
            obsToRob = self.currentPosition - obstacle

            robToObs = obstacle - self.currentPosition

            if self.length(obsToRob) > self.repulsiveRadius:
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
                    * (1.0 / self.length(obsToRob) - 1.0 / 2.0)
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
                    * ((1.0 / self.length(robToObs) - 1.0 / 2.0) ** 2)
                    * self.length(obsToRob)
                )
                total = total + rep + rep2

        for obstacle in self.extraBlue:
            obsToRob = self.currentPosition - obstacle

            robToObs = obstacle - self.currentPosition
            if self.length(obsToRob) > 2.0:
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
                    * (1.0 / self.length(obsToRob) - 1.0 / 2.0)
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
                    * ((1.0 / self.length(robToObs) - 1.0 / 2.0) ** 2)
                    * self.length(obsToRob)
                )
                total = total + rep + rep2

        for obstacle in self.obstacles:
            obsToRob = self.currentPosition - obstacle

            robToObs = obstacle - self.currentPosition
            if self.length(obsToRob) > self.repulsiveRadius:
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
                    * (1.0 / self.length(obsToRob) - 1.0 / self.repulsiveRadius)
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
                    * ((1.0 / self.length(robToObs) - 1.0 / self.repulsiveRadius) ** 2)
                    * self.length(obsToRob)
                )
                total = total + rep + rep2

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
            for obstacle in self.obsYellow:
                circle = Circle(
                    xy=(obstacle[0], obstacle[1]),
                    radius=self.repulsiveRadius,
                    alpha=0.3,
                )
                subplot.add_patch(circle)
                circle.set_facecolor("yellow")
                subplot.plot(obstacle[0], obstacle[1], "xk")
            for obstacle in self.obsBlue:
                circle = Circle(
                    xy=(obstacle[0], obstacle[1]),
                    radius=self.repulsiveRadius,
                    alpha=0.3,
                )
                subplot.add_patch(circle)
                circle.set_facecolor("blue")
                subplot.plot(obstacle[0], obstacle[1], "xk")
            for obstacle in self.obstacles:
                if obstacle not in self.obsBlue and obstacle not in self.obsYellow:
                    circle = Circle(
                        xy=(obstacle[0], obstacle[1]),
                        radius=self.repulsiveRadius,
                        alpha=0.3,
                    )
                    subplot.add_patch(circle)
                    circle.set_facecolor("grey")
                    subplot.plot(obstacle[0], obstacle[1], "xk")
            self.obsBlue = self.distanceToObstacles(self.obsBlue)
            if len(self.obsBlue) > 1:
                self.extraBlue = np.array([(self.obsBlue[0] + self.obsBlue[1]) / 2])
                for i, _ in enumerate(self.obsBlue[1:-1], start=1):
                    midpoint = (self.obsBlue[i] + self.obsBlue[i + 1]) / 2
                    self.extraBlue = np.vstack((self.extraBlue, midpoint))
            for obstacle in self.extraBlue:
                circle = Circle(
                    xy=(obstacle[0], obstacle[1]),
                    radius=self.repulsiveRadius / 2,
                    alpha=0.3,
                )
                subplot.add_patch(circle)
                circle.set_facecolor((0, 0, 0.8))
                subplot.plot(obstacle[0], obstacle[1], "xk")
            # sortedYellow = self.distanceToObstacles(self.obsYellow)
            if len(self.obsYellow) > 1:
                self.extraYellow = np.array([(self.obsYellow[0] + self.obsYellow[1]) / 2])
                for i, _ in enumerate(self.obsYellow[1:-1], start=1):
                    midpoint = (self.obsYellow[i] + self.obsYellow[i + 1]) / 2
                    self.extraYellow = np.vstack((self.extraYellow, midpoint))
            for obstacle in self.extraYellow:
                circle = Circle(
                    xy=(obstacle[0], obstacle[1]),
                    radius=self.repulsiveRadius / 2,
                    alpha=0.3,
                )
                subplot.add_patch(circle)
                circle.set_facecolor((0.8, 0.8, 0))
                subplot.plot(obstacle[0], obstacle[1], "xk")

        while self.iterations < self.maxIterations:
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
