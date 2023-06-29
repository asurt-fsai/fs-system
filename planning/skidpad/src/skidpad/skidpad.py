#!/usr/bin/python3
"""
Skidpad class is used to visulaize skid-pad and the waypoints of it,
in addition to calculate waypoints
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Tuple
import numpy.typing as npt


class Skidpad:  # pylint: disable=R0902
    """
    Skidpad class represents a skidpad. It is initialized with parameters such as
    the distance between the two circular paths,the inner and outer radius of the circles,
    and the lengths of the lines at the top and bottom of the skidpad.
    """

    def __init__(
        self,
        distance: float,
        innerRadius: float,
        outerRadius: float,
        lengthOfLineB: float,
        lengthOfLineT: float,
    ) -> None:
        """
        Initialize the Skidpad object.

        Parameters:
        ------------
        distance : float
            Distance between the two circles' centers.
        innerRadius : float
            Inner radius of the circles.
        outerRadius : float
            Outer radius of the circles.
        lengthOfLineB : float
            Length of the bottom line.
        lengthOfLineT : float
            Length of the top line.
        """

        self.distance = distance
        self.innerRadius = innerRadius
        self.outerRadius = outerRadius
        self.lengthOfLineB = lengthOfLineB
        self.lengthOfLineT = lengthOfLineT
        self.center1 = np.array(
            [
                -self.distance / 2,
                self.outerRadius * np.sin(np.arccos((self.innerRadius) / self.outerRadius))
                + self.lengthOfLineB,
            ]
        )
        self.center2 = np.array(
            [
                self.distance / 2,
                self.outerRadius * np.sin(np.arccos((self.innerRadius) / self.outerRadius))
                + self.lengthOfLineB,
            ]
        )
        self.waypoints = np.empty((0, 2))
        self.calculate()

    def calculate(self) -> None:  # pylint: disable=R0914
        """
        Calculate the waypoints of the skidpad path.

        Returns:
        -----------
        None
        """

        thetaWayR = np.linspace(np.pi, -np.pi, 100)
        thetaWayL = np.linspace(0, 2 * np.pi, 100)
        waypointsR = np.empty((0, 2))
        waypointsL = np.empty((0, 2))
        line1Waypoints = np.empty((0, 2))
        line2Waypoints = np.empty((0, 2))

        # Calculate waypoints for the right circle
        for theta in thetaWayR:
            waypointsXR = self.center2[0] + (self.outerRadius + self.innerRadius) / 2 * np.cos(
                theta
            )
            waypointsLR = self.center2[1] + (self.outerRadius + self.innerRadius) / 2 * np.sin(
                theta
            )
            waypointsR = np.vstack((waypointsR, [waypointsXR, waypointsLR]))

        # Calculate waypoints for the left circle
        for theta in thetaWayL:
            waypointsXL = self.center1[0] + (self.outerRadius + self.innerRadius) / 2 * np.cos(
                theta
            )
            waypointsYL = self.center1[1] + (self.outerRadius + self.innerRadius) / 2 * np.sin(
                theta
            )
            waypointsL = np.vstack((waypointsL, [waypointsXL, waypointsYL]))

        step = math.sqrt(
            math.pow(waypointsL[1, 0] - waypointsL[0, 0], 2)
            + math.pow(waypointsL[1, 1] - waypointsL[0, 1], 2)
        )
        theta = np.arccos((self.innerRadius) / self.outerRadius)
        line1Y = np.linspace(
            self.center1[1],
            self.center1[1] + self.outerRadius * np.sin(theta) + self.lengthOfLineT,
            int(
                abs(self.center2[1] + self.outerRadius * np.sin(theta) + self.lengthOfLineT) / step
            ),
        )
        line2Y = np.linspace(
            0,
            self.center1[1],
            int(
                abs(self.center1[1] + self.outerRadius * np.sin(theta) + self.lengthOfLineB) / step
            ),
        )
        lineX = (self.center1[0] + self.center2[0]) / 2

        # Calculate waypoints for the top and bottom lines
        for lineY in line1Y:
            line1Waypoints = np.vstack((line1Waypoints, [lineX, lineY]))
        for lineY in line2Y:
            line2Waypoints = np.vstack((line2Waypoints, [lineX, lineY]))

        lastPoint = line2Waypoints[-1]
        reshapedLastPoint = np.reshape(lastPoint, (1, lastPoint.shape[0]))

        # Concatenate all waypoints
        self.waypoints = np.concatenate(
            (
                line2Waypoints,
                waypointsR,
                waypointsR,
                reshapedLastPoint,
                waypointsL,
                waypointsL,
                line1Waypoints,
            )
        )

    def getWaypoints(self) -> npt.NDArray[np.float64]:
        """
        Get the waypoints of the skidpad path.

        Returns:
        -----------
        numpy.ndarray
            Array of waypoints representing the skidpad path.
        """

        return self.waypoints

    def getIntersections(  # pylint: disable=R0914
        self,
        center0X: float,
        center0Y: float,
        radius0: float,
        center1X: float,
        center1Y: float,
        radius1: float,
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        Calculate the intersections of two circles.

        Parameters:
        ------------
        center0X : float
            x-coordinate of the center of the first circle.
        center0Y : float
            y-coordinate of the center of the first circle.
        radius0 : float
            Radius of the first circle.
        center1X : float
            x-coordinate of the center of the second circle.
        center1Y : float
            y-coordinate of the center of the second circle.
        radius1 : float
            Radius of the second circle.

        Returns:
        -----------
        tuple or None
            Tuple containing the (x, y) coordinates of the intersection points if they exist,
            or None if the circles do not intersect.
        """

        euclideanDistance = math.sqrt((center1X - center0X) ** 2 + (center1Y - center0Y) ** 2)

        # Non-intersecting
        if euclideanDistance > radius0 + radius1:
            return None

        # One circle within the other
        if euclideanDistance < abs(radius0 - radius1):
            return None

        # Coincident circles
        if euclideanDistance == 0 and radius0 == radius1:
            return None

        distanceAlongLine = (radius0**2 - radius1**2 + euclideanDistance**2) / (
            2 * euclideanDistance
        )
        distancePerpendicular = math.sqrt(radius0**2 - distanceAlongLine**2)

        center2X = center0X + distancePerpendicular * (center1X - center0X) / euclideanDistance
        center2Y = center0Y + distancePerpendicular * (center1Y - center0Y) / euclideanDistance

        intersectionPoint1X = (
            center2X + distancePerpendicular * (center1Y - center0Y) / euclideanDistance
        )
        intersectionPoint1Y = (
            center2Y - distancePerpendicular * (center1X - center0X) / euclideanDistance
        )
        intersectionPoint2X = (
            center2X - distancePerpendicular * (center1Y - center0Y) / euclideanDistance
        )
        intersectionPoint2Y = (
            center2Y + distancePerpendicular * (center1X - center0X) / euclideanDistance
        )
        return (
            intersectionPoint1X,
            intersectionPoint1Y,
            intersectionPoint2X,
            intersectionPoint2Y,
        )

    def drawSkidpad(self) -> None:  # pylint: disable=R0914
        """
        Draw the skidpad path using Matplotlib.
        """
        theta = np.linspace(0, 2 * np.pi, 1000)
        inner1 = np.array(
            [
                self.center1[0] + self.innerRadius * np.cos(theta),
                self.center1[1] + self.innerRadius * np.sin(theta),
            ]
        )
        inner2 = np.array(
            [
                self.center2[0] + self.innerRadius * np.cos(theta),
                self.center2[1] + self.innerRadius * np.sin(theta),
            ]
        )
        outer1 = np.array(
            [
                self.center1[0] + self.outerRadius * np.cos(theta),
                self.center1[1] + self.outerRadius * np.sin(theta),
            ]
        )
        outer2 = np.array(
            [
                self.center2[0] + self.outerRadius * np.cos(theta),
                self.center2[1] + self.outerRadius * np.sin(theta),
            ]
        )
        theta1 = np.arccos((self.innerRadius) / self.outerRadius)
        lineTopLeft = np.array(
            [
                self.center1[0] + self.innerRadius,
                self.center1[1] + self.outerRadius * np.sin(theta1),
            ]
        )
        lineTopRight = np.array(
            [
                self.center2[0] - self.innerRadius,
                self.center2[1] + self.outerRadius * np.sin(theta1),
            ]
        )
        lineBottomLeft = np.array(
            [
                self.center1[0] + self.innerRadius,
                self.center1[1] - self.outerRadius * np.sin(theta1),
            ]
        )
        lineBottomRight = np.array(
            [
                self.center2[0] - self.innerRadius,
                self.center2[1] - self.outerRadius * np.sin(theta1),
            ]
        )
        intersections = self.getIntersections(
            self.center1[0],
            self.center1[1],
            self.outerRadius,
            self.center2[0],
            self.center2[1],
            self.outerRadius,
        )
        if intersections:
            intersection1X, intersection1Y, intersection2X, intersection2Y = intersections

            indices1 = (intersection1X - self.distance / 2 < outer1[0]) & (
                outer1[0] < intersection2X + self.distance / 2
            ) & (intersection1Y < outer1[1]) & (outer1[1] < intersection2Y) & (
                outer1[0] != intersection1X
            ) | (
                (outer1[0] > lineTopLeft[0]) & (outer1[0] < lineTopRight[0])
            )

            indices2 = (intersection1X - self.distance / 2 < outer2[0]) & (
                outer2[0] < intersection2X + self.distance / 2
            ) & (intersection1Y < outer2[1]) & (outer2[1] < intersection2Y) & (
                outer2[0] != intersection1X
            ) | (
                (outer2[0] > lineTopLeft[0]) & (outer2[0] < lineTopRight[0])
            )

            # Plot the circles
            plt.plot(inner1[0], inner1[1])
            plt.plot(inner2[0], inner2[1])

            # Plot the trimmed region
            plt.plot(outer1[0][~indices1], outer1[1][~indices1], "b.")
            plt.plot(outer2[0][~indices2], outer2[1][~indices2], "b.")

        else:
            # Plot the circles if there are no intersections
            plt.plot(inner1[0], inner1[1])
            plt.plot(inner2[0], inner2[1])
            plt.plot(outer1[0], outer1[1], "b.")
            plt.plot(outer2[0], outer2[1], "b.")

        # Plot the centers
        plt.plot(self.waypoints[:, 0], self.waypoints[:, 1], "ro")
        plt.plot(self.center1[0], self.center1[1], "ro")
        plt.plot(self.center2[0], self.center2[1], "bo")
        plt.plot(
            [lineTopLeft[0], lineTopLeft[0]],
            [lineTopLeft[1], lineTopLeft[1] + self.lengthOfLineT],
            "b-",
            linewidth=4.0,
        )
        plt.plot(
            [lineTopRight[0], lineTopRight[0]],
            [lineTopRight[1], lineTopRight[1] + self.lengthOfLineT],
            "b-",
            linewidth=4.0,
        )
        plt.plot(
            [lineBottomLeft[0], lineBottomLeft[0]],
            [lineBottomLeft[1], lineBottomLeft[1] - self.lengthOfLineB],
            "b-",
            linewidth=4.0,
        )
        plt.plot(
            [lineBottomRight[0], lineBottomRight[0]],
            [lineBottomRight[1], lineBottomRight[1] - self.lengthOfLineB],
            "b-",
            linewidth=4.0,
        )

        plt.grid(True)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.show()


# distance = 18.25
#  Radius of the circles
# innerRadius = 15.25 / 2
# outerRadius = 21.25 / 2
# len_of_line_t = 3.0
# lengthOfLineB = 6.0


# skidpad = Skidpad(distance, innerRadius, outer_radius, lengthOfLineB, len_of_line_t)
# skidpad.drawSkidpad()
# waypoints = skidpad.getWaypoints()

# for i in range(10,waypoints.shape[0]):
#     plt.scatter(waypoints[i][0], waypoints[i][1])
#     plt.pause(0.01)
