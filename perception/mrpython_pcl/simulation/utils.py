"""
Contains utility functions for lidar simulation.
"""
# mypy: ignore-errors
from typing import Tuple

import numpy as np

import matplotlib.pyplot as plt


def getConeEquation(theta: float, distance: float, height: float, radius: float) -> Tuple[float]:
    """
    Generates the cone center and parameters.

    Parameters:
    -----------
        theta: float
            horizontal angle in degrees
        distance: float
            distance to the center of the cone in meters
        height: float
            cone height in meters
        radius: float
            radius of the cone base in meters

    Returns:
    --------
        Tuple[float]
            the x position of the center of the cone base
            the y position of the center of the cone base
            the cone parameter
    """
    theta = np.deg2rad(theta)
    x = distance * np.cos(theta)
    y = distance * np.sin(theta)
    param = radius**2 / height**2

    return x, y, height, param


def getLineEquation(origin: Tuple[float], alpha: float, theta: float) -> Tuple[float]:
    """
    Generates the line equation.

    Parameters:
    -----------
        origin: Tuple[float]
            the position of the reference point in meters
        alpha: float
            the vertical angle in degrees
        theta: float
            the horizontal angle in degrees

    Returns:
    --------
        Tuple[float]
            originX representing x0
            originY representing y0
            originZ representing z0
            directionX representing the direction in the x axis
            directionY representing the direction in the y axis
            directionZ representing the direction in the z axis
    """
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    directionX = np.cos(theta) * np.cos(alpha)
    directionY = np.sin(theta) * np.cos(alpha)
    directionZ = np.sin(alpha)

    return *origin, directionX, directionY, directionZ


def computeIntersection(cone: Tuple[float], line: Tuple[float]) -> Tuple[float]:
    # pylint: disable=too-many-locals,invalid-name
    """
    Computes the intersection between the cone and the line (LiDAR ray).

    Parameters:
    -----------
        cone: Tuple[float]
            represents the cone equation
        line: Tuple[float]
            represents the line equation

    Returns:
    --------
        Tuple[float]
            The x,y,z of the nearest intersection point.
    """
    # Extract coordinates of the center of the cone
    centerX, centerY, centerZ, coneParameter = cone

    # Extract coordinates of the line origin
    originX, originY, originZ, directionX, directionY, directionZ = line

    # Calculate coefficients for the quadratic equation
    s1 = originX - centerX
    s2 = originY - centerY
    s3 = originZ - centerZ

    a = directionX**2 + directionY**2 - coneParameter * directionZ**2
    b = 2 * (directionX * s1 + directionY * s2 - coneParameter * directionZ * s3)
    c = s1**2 + s2**2 - coneParameter * s3**2

    # Calculate discriminant
    discriminant = b**2 - 4 * a * c

    # Check if the line intersects the cone
    if discriminant < 0:
        # No intersection
        return None

    # Compute the parameter values for the intersection points
    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    # Compute the intersection points
    intersection1 = (
        originX + t1 * directionX,
        originY + t1 * directionY,
        originZ + t1 * directionZ,
    )
    intersection2 = (
        originX + t2 * directionX,
        originY + t2 * directionY,
        originZ + t2 * directionZ,
    )

    # Calculate distances from line origin to the intersection points
    distance1 = np.sqrt(
        (intersection1[0] - originX) ** 2
        + (intersection1[1] - originY) ** 2
        + (intersection1[2] - originZ) ** 2
    )
    distance2 = np.sqrt(
        (intersection2[0] - originX) ** 2
        + (intersection2[1] - originY) ** 2
        + (intersection2[2] - originZ) ** 2
    )

    # Return the nearest intersection point
    if distance1 < distance2:
        return intersection1
    return intersection2


def visualizeLineAndCone(
    lineOrigin: Tuple[float],
    lineDirection: Tuple[float],
    coneCenter: Tuple[float],
    coneRadius: float,
) -> None:
    """
    visualizes the line and the cone

    Parameters:
    ------------
        lineOrigin: Tuple[float]
            the position of the reference point
        lineDirection: Tuple[float]
            the direction of the line
        coneCenter: Tuple[float]
            the position of the center of the cone base
        coneRadius: float
            the radius of the cone base
    """
    # Generate points along the line
    x = np.linspace(-1000, 1000, 100)
    y = (lineDirection[1] / lineDirection[0]) * (x - lineOrigin[0]) + lineOrigin[1]

    # Generate points along the circumference of the cone base
    theta = np.linspace(0, 2 * np.pi, 100)
    conePointX = coneCenter[0] + coneRadius * np.cos(theta)
    conePointY = coneCenter[1] + coneRadius * np.sin(theta)

    # Plotting the line and the cone
    plt.plot(x, y, label="Line")
    plt.plot(conePointX, conePointY, label="Cone", marker="x")
    plt.plot(0, 0, label="origin", marker="o")

    # Set plot limits and labels
    plt.xlim(-40, 40)
    plt.ylim(-40, 40)
    plt.xlabel("x")
    plt.ylabel("y")

    # Add legend
    plt.legend()

    # Show the plot
    plt.show()
