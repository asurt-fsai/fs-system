#!/usr/bin/python3
"""
Skidpad class is used to visulaize skid-pad and the waypoints of it,
in addition to calculate waypoints
"""
import math
from typing import Optional, Tuple
import numpy.typing as npt
import numpy as np
import matplotlib.pyplot as plt


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

    def getWaypoints(self, step: float) -> npt.NDArray[np.float64]:  # pylint: disable=R0902, R0914
        """
        Get the waypoints of the skidpad path.
        Parameters:
        ------------
        step:float
        distance between each two points of the waypoints

        Returns:
        -----------
        numpy.ndarray
            Array of waypoints representing the skidpad path.
        """

        thetaWayR = np.linspace(
            np.pi,
            -np.pi,
            int(
                2
                * np.pi
                / np.arccos(
                    (2 * ((self.outerRadius + self.innerRadius) / 2) ** 2 - step**2)
                    / (2 * ((self.outerRadius + self.innerRadius) / 2) ** 2)
                )
            ),
        )
        thetaWayL = np.linspace(
            0,
            2 * np.pi,
            int(
                2
                * np.pi
                / np.arccos(
                    (2 * ((self.outerRadius + self.innerRadius) / 2) ** 2 - step**2)
                    / (2 * ((self.outerRadius + self.innerRadius) / 2) ** 2)
                )
            ),
        )
        waypoints = np.empty((0, 2))
        waypointsR = np.empty((0, 2))
        waypointsL = np.empty((0, 2))
        line1Waypoints = np.empty((0, 2))
        line2Waypoints = np.empty((0, 2))

        # Calculate waypoints for the right circle
        for theta in thetaWayR:
            waypointsXR = self.center2[0] + (self.outerRadius + self.innerRadius) / 2 * np.cos(
                theta
            )
            waypointsYR = self.center2[1] + (self.outerRadius + self.innerRadius) / 2 * np.sin(
                theta
            )
            waypointsR = np.vstack((waypointsR, [waypointsXR, waypointsYR]))

        # Calculate waypoints for the left circle
        for theta in thetaWayL:
            waypointsXL = self.center1[0] + (self.outerRadius + self.innerRadius) / 2 * np.cos(
                theta
            )
            waypointsYL = self.center1[1] + (self.outerRadius + self.innerRadius) / 2 * np.sin(
                theta
            )
            waypointsL = np.vstack((waypointsL, [waypointsXL, waypointsYL]))

        theta = np.arccos((self.innerRadius) / self.outerRadius)
        line1Y = np.linspace(
            self.center1[1],
            self.center1[1] + self.outerRadius * np.sin(theta) + self.lengthOfLineT,
            math.ceil((self.outerRadius * np.sin(theta) + self.lengthOfLineT) / step),
        )
        line2Y = np.linspace(
            0,
            self.center1[1],
            math.ceil(self.center1[1] / step),
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
        waypoints = np.concatenate(
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
        return waypoints

    def getConesPosition(  # pylint: disable=R0902, R0914
        self, step: float
    ) -> npt.NDArray[np.float64]:
        """
        Get the position of skidpad cones .
        Parameters:
        ------------
        step:float
        distance between two adjacent cones
        Returns:
        -----------
        numpy.ndarray
            Array of cone positions .
        """
        conesPosition = np.empty((0, 2))
        lineTopLeftCones = np.empty((0, 2))
        lineTopRightCones = np.empty((0, 2))
        linebottomLeftCones = np.empty((0, 2))
        linebottomRightCones = np.empty((0, 2))
        thetaInner = np.linspace(
            0,
            2 * np.pi,
            math.ceil(
                2
                * np.pi
                / np.arccos((2 * self.innerRadius**2 - step**2) / (2 * self.innerRadius**2))
            ),
        )
        thetaOuter = np.linspace(
            0,
            2 * np.pi,
            math.ceil(
                2
                * np.pi
                / np.arccos((2 * self.outerRadius**2 - step**2) / (2 * self.outerRadius**2))
            ),
        )

        inner1 = np.array(
            [
                self.center1[0] + self.innerRadius * np.cos(thetaInner),
                self.center1[1] + self.innerRadius * np.sin(thetaInner),
            ]
        )
        inner2 = np.array(
            [
                self.center2[0] + self.innerRadius * np.cos(thetaInner),
                self.center2[1] + self.innerRadius * np.sin(thetaInner),
            ]
        )
        outer1 = np.array(
            [
                self.center1[0] + self.outerRadius * np.cos(thetaOuter),
                self.center1[1] + self.outerRadius * np.sin(thetaOuter),
            ]
        )
        outer2 = np.array(
            [
                self.center2[0] + self.outerRadius * np.cos(thetaOuter),
                self.center2[1] + self.outerRadius * np.sin(thetaOuter),
            ]
        )

        lineLeftX = self.center1[0] + self.innerRadius
        lineRightX = self.center2[0] - self.innerRadius
        lineTopY = np.linspace(
            self.center1[1]
            + self.outerRadius * np.sin(np.arccos((self.innerRadius) / self.outerRadius)),
            self.center1[1]
            + self.outerRadius * np.sin(np.arccos((self.innerRadius) / self.outerRadius))
            + self.lengthOfLineT,
            math.ceil(self.lengthOfLineT / step),
        )

        lineBottomY = np.linspace(
            0,
            self.lengthOfLineB,
            math.ceil(self.lengthOfLineB / step),
        )
        for lineY in lineTopY:
            lineTopLeftCones = np.vstack((lineTopLeftCones, [lineLeftX, lineY]))
            lineTopRightCones = np.vstack((lineTopRightCones, [lineRightX, lineY]))
        for lineY in lineBottomY:
            linebottomLeftCones = np.vstack((linebottomLeftCones, [lineLeftX, lineY]))
            linebottomRightCones = np.vstack((linebottomRightCones, [lineRightX, lineY]))

        intersections = self.getIntersections(
            self.center1[0],
            self.center1[1],
            self.outerRadius,
            self.center2[0],
            self.center2[1],
            self.outerRadius,
        )

        if intersections:
            (
                intersection1X,
                intersection1Y,
                intersection2X,
                intersection2Y,
            ) = intersections

            indices1 = (intersection1X - self.distance / 2 < outer1[0]) & (
                outer1[0] < intersection2X + self.distance / 2
            ) & (intersection1Y < outer1[1]) & (outer1[1] < intersection2Y) & (
                outer1[0] != intersection1X
            ) | (
                (outer1[0] > lineLeftX) & (outer1[0] < lineRightX)
            )

            indices2 = (intersection1X - self.distance / 2 < outer2[0]) & (
                outer2[0] < intersection2X + self.distance / 2
            ) & (intersection1Y < outer2[1]) & (outer2[1] < intersection2Y) & (
                outer2[0] != intersection1X
            ) | (
                (outer2[0] > lineLeftX) & (outer2[0] < lineRightX)
            )

            # Concatenate all Cones Position

            conesPosition = np.concatenate(
                (
                    linebottomLeftCones,
                    linebottomRightCones,
                    np.array([inner1[0][0:-1], inner1[1][0:-1]]).T,
                    np.array([inner2[0][0:-1], inner2[1][0:-1]]).T,
                    np.array([outer1[0][~indices1], outer1[1][~indices1]]).T,
                    np.array([outer2[0][~indices2], outer2[1][~indices2]]).T,
                    lineTopLeftCones,
                    lineTopRightCones,
                )
            )
        else:
            conesPosition = np.concatenate(
                (
                    linebottomLeftCones,
                    linebottomRightCones,
                    np.array([inner1[0][0:-1], inner1[1][0:-1]]).T,
                    np.array([inner2[0][0:-1], inner2[1][0:-1]]).T,
                    np.array([outer1[0][0:-1], outer1[1][0:-1]]).T,
                    np.array([outer2[0][0:-1], outer2[1][0:-1]]).T,
                    lineTopLeftCones,
                    lineTopRightCones,
                )
            )
        return conesPosition

    def drawSkidpadCones(self, step: float) -> None:  # pylint: disable=R0914
        """
        Draw the skidpad path using Matplotlib.
        Parameters:
        ------------
        step:float
        Returns:
        -----------
        None
        """
        conePositions = self.getConesPosition(step)
        plt.plot(conePositions[:, 0], conePositions[:, 1], "b.")
        plt.grid(True)
        plt.show()

    def drawSkidpadwaypoints(self, step: float) -> None:  # pylint: disable=R0914
        """
        Draw the skidpad path using Matplotlib.
        Parameters:
        ------------
        step:float
        distance between two adjacent ponints
        Returns:
        -----------
        None
        """
        waypoints = self.getWaypoints(step)
        plt.plot(waypoints[:, 0], waypoints[:, 1], "b.")
        plt.grid(True)
        plt.show()


# distance = 18.25
# #  Radius of the circles
# innerRadius = 15.25 / 2
# outerRadius = 21.25 / 2
# len_of_line_t = 3.0
# lengthOfLineB = 6.0


# skidpad = Skidpad(distance, innerRadius, outerRadius, lengthOfLineB, len_of_line_t)
# # skidpad.drawSkidpad()
# waypoints = skidpad.getWaypoints(0.1)
# cones = skidpad.getConesPosition(0.1)
# skidpad.drawSkidpadCones(0.5)
# # skidpad.drawSkidpadwaypoints(0.5)


# for i in range(10,waypoints.shape[0]):
#     plt.scatter(waypoints[i][0], waypoints[i][1])
#     plt.pause(0.01)
