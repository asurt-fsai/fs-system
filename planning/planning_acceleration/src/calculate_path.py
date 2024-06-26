'''
Description: This module calculates the path if 2 or less cones are detected on both sides
'''

from typing import List
import math
import numpy as np
from numpy.typing import NDArray

from src.cone_types import ConeTypes

FloatArray = NDArray[np.float_]


class CalculatePath:
    """
    Class that is responsible for calculating the path
    when 2 or less cones are detected on both sides
    """
    def __init__(self,
        carPosition: FloatArray,
        carDirection: np.float_,
        cones: List[FloatArray]
    ) -> None:
        self.carPosition = carPosition
        self.carDirection = carDirection
        self.cones = cones

    def getPath(
        self,
    ) -> FloatArray:
        '''
        calculates the path
        '''
        if len(self.cones[ConeTypes.left]) >= 2:
            sortedCones = sortCones(self.cones[ConeTypes.left], self.carPosition)
            return self.getPathFrom2PointsSameSide(sortedCones, ConeTypes.BLUE)
        if len(self.cones[ConeTypes.right]) >= 2:
            sortedCones = sortCones(self.cones[ConeTypes.right], self.carPosition)
            return self.getPathFrom2PointsSameSide(sortedCones, ConeTypes.YELLOW)
        if len(self.cones[ConeTypes.left]) == 1 and len(self.cones[ConeTypes.right]) == 1:
            return self.getPathFrom2DifferentPoints()
        if len(self.cones[ConeTypes.left]) == 1:
            return self.pathToAvoidCone(self.cones[ConeTypes.left], ConeTypes.BLUE)
        if len(self.cones[ConeTypes.right]) == 1:
            return self.pathToAvoidCone(self.cones[ConeTypes.right], ConeTypes.YELLOW)
        return createStraightPath(self.carPosition, self.carDirection, 10)

    def pathToAvoidCone(
        self,
        cone: FloatArray,
        color: ConeTypes
    ) -> FloatArray:
        '''
        Calculate path if one cone is found
        '''
        if color == ConeTypes.YELLOW:
            if self.isConeOnRightSide(cone):
                return createStraightPath(self.carPosition, self.carDirection, 10)
            newDirection = np.float_(angleFrom2dVector(cone - self.carPosition)) + 0.4
            return createStraightPath(self.carPosition, newDirection, 10)
        # if cone is BLUE
        if not self.isConeOnRightSide(cone):
            return createStraightPath(self.carPosition, self.carDirection, 10)
        newDirection = np.float_(angleFrom2dVector(cone - self.carPosition)) - 0.4
        return createStraightPath(self.carPosition, newDirection, 10)

    def isConeOnRightSide(
        self,
        conePosition: FloatArray
    ) -> bool:
        """
        Checks if a cone is on the car's right side based on positions and direction.

        Args:
            car_position: A 2D vector (x, y) representing the car's position.
            car_direction: The car's direction in degrees (0: right, 90: up, 180: left, 270: down).
            cone_position: A 2D vector (x, y) representing the cone's position.

        Returns:
            True if the cone is on the car's right side, False otherwise.
        """
        carDirectionVector = angleToVector(self.carDirection)

        carToCone = conePosition - self.carPosition

        crossProduct = np.cross(carDirectionVector, carToCone)

        if crossProduct < 0:
            return True
        return False

    def getPathFrom2DifferentPoints(self) -> FloatArray:
        '''
        get straight path from car position passing between the two points
        '''
        point = (self.cones[ConeTypes.left][0] + self.cones[ConeTypes.right][0]) / 2
        direction = point - self.carPosition
        directionAngle = np.float_(angleFrom2dVector(direction))
        path = createStraightPath(self.carPosition, directionAngle, 10)
        return path

    def getPathFrom2PointsSameSide(
        self,
        cones: FloatArray,
        color: ConeTypes
    ) -> FloatArray:
        '''
        Calculates path with only 2 points given
        '''
        pointsDirection = cones[1] - cones[0]
        directionConeToPoint: FloatArray
        if color == ConeTypes.BLUE:
            directionConeToPoint = np.array([pointsDirection[1], -pointsDirection[0]])
        else:
            directionConeToPoint = np.array([-pointsDirection[1], pointsDirection[0]])
        directionConeToPoint = directionConeToPoint / np.linalg.norm(directionConeToPoint)
        points: FloatArray = cones + directionConeToPoint * 1.5
        pointsDirectionAngle: np.float_ = np.float_(angleFrom2dVector(pointsDirection))
        path = createStraightPath(points[0], pointsDirectionAngle, 10)
        pathExtention = self.connectPathToCar(path[0])
        path = np.row_stack((pathExtention, path))
        return path

    def connectPathToCar(self, firstPoint: FloatArray) -> FloatArray:
        """
        Connect the path update to the current path of the car. This is done by
        calculating the distance between the last point of the path update and the
        current position of the car. The path update is then shifted by this distance.
        """
        numOfPoints = math.floor(distanceBetweenPoints(self.carPosition, firstPoint))
        direction = np.float_(angleFrom2dVector(firstPoint - self.carPosition))

        pathUpdate = createStraightPath(self.carPosition, direction, numOfPoints)

        return pathUpdate


def sortCones(cones: FloatArray, carPosition: FloatArray) -> FloatArray:
    '''
    sorts the cones
    '''
    sortedCones: FloatArray = cones
    if distanceBetweenPoints(carPosition, cones[0]) > distanceBetweenPoints(carPosition, cones[1]):
        sortedCones[0] = cones[1]
        sortedCones[1] = cones[0]
    return sortedCones

def createStraightPath(startPoint: FloatArray, direction: np.float_, numPoints: int) -> FloatArray:
    """
    Creates a list of points along a straight path with a given distance between points.

    Args:
        start_point: A tuple (x, y) representing the starting point.
        direction: Direction in degrees (0: right, 90: up, 180: left, 270: down).
        num_points: Total number of points in the path (including the starting point).

    Returns:
        A list of tuples (x, y) representing the points on the path.
    """
    path: FloatArray = np.array([startPoint])
    x, y = startPoint
    # Calculate step vector components
    deltaX = math.cos(direction)
    deltaY = math.sin(direction)

    # Generate subsequent points
    for _ in range(1, numPoints):
        x += deltaX
        y += deltaY
        newPoint = [x,y]
        path = np.row_stack((path, newPoint))

    return path

def distanceBetweenPoints(point1: FloatArray, point2: FloatArray) -> np.float_:
    """
    Calculates the distance between two 2D points.

    Args:
        point1: A tuple (x, y) representing the first point.
        point2: A tuple (x, y) representing the second point.

    Returns:
        The distance between the two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    return np.float_(math.sqrt((x2 - x1)**2 + (y2 - y1)**2))

def angleToVector(angle: np.float_) -> FloatArray:
    """
    Converts an angle in radians to a 2D unit vector.

    Args:
        angle: The angle in radians.

    Returns:
        A 2D unit vector representing the direction of the angle.
    """

    x = math.cos(angle)
    y = math.sin(angle)

    # Normalize to create a unit vector
    magnitude = math.sqrt(x**2 + y**2)
    x /= magnitude
    y /= magnitude
    unitVector = np.array([x / magnitude, y / magnitude], dtype=np.float_)

    return unitVector

def angleFrom2dVector(vecs: np.ndarray) -> np.ndarray:
    """
    Calculates the angle of each vector in `vecs`. If `vecs` is just a single 2d vector
    then one angle is calculated and a scalar is returned

    Args:
        vecs (np.array): The vectors for which the angle is calculated

    Raises:
        ValueError: If `vecs` has the wrong shape a ValueError is raised

    Returns:
        np.array: The angle of each vector in `vecs`
    """
    assert vecs.shape[-1] == 2, "vecs must be a 2d vector"

    vecsFlat = vecs.reshape(-1, 2)

    angles = np.arctan2(vecsFlat[:, 1], vecsFlat[:, 0])
    returnValue = angles.reshape(vecs.shape[:-1])

    return returnValue
