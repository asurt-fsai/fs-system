#!/usr/bin/env python
"""
indication for the interpreter that should be used to run the script.
"""
# pylint: disable=too-many-instance-attributes

import typing
import time
from threading import Lock
from typing import Callable, List
from typing import Any
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray

# from tf_helper import *
from tf.transformations import euler_from_quaternion

mutex = Lock()


def mutexLock(func: "Callable[...,Any]") -> "Callable[...,Any]":
    """
    The function is used to secure shared resources between functions by creating a mutex
    once the function is called and releasing it after function exceution
    """

    def newFunc(*args: "int", **kwargs: "int") -> "Any":
        with mutex:
            val = None
            val = func(*args, **kwargs)
            return val

    return newFunc


def timeit(func: "Callable[...,Any]") -> "Callable[...,Any]":
    """
    returns the execution time of a function
    """

    def newFunc(*args: "int", **kwargs: "int") -> "Any":
        tic = time.time()
        val = None
        try:
            val = func(*args, **kwargs)
        finally:
            pass
        toc = time.time()
        print(func.__name__, ":", (toc - tic) * 1000, "ms")
        return val

    return newFunc


class SlamData:
    """
    SlamData is a class which has the attributes required for
    1- Clustring the cones such as: minimum number of times the cone has been detected,
       the minumim distance between 2 cones to be considered separate

    2- Determining number of laps completed and distance travel as it has attributes
       that stores the values of first position,last position and distance travel.
       Moreover, it has an attribute used to determine if the current position is far from the
       first position

    it has 5 Methods save_odometry,parse,add_cones,cluster_cones,get_cones

    """

    def __init__(self) -> None:
        # self.didClosure = True
        # self.gtPositions: NDArray[Any] = np.array([])
        self.clusteredCones: List["float"]
        self.unclusteredCones: List[List["float"]] = []
        self.clusteredColors: List[List["int"]] = []
        self.clusteredConescount: List["int"] = []
        # self.conePoseids: NDArray[Any] = np.array([])
        # self.pointclouds: NDArray[Any] = np.array([])
        self.lastPosition = np.zeros(3)
        # self.counter = 0
        # self.lastAddedpose = None
        self.distTravelled: float = 0
        self.lapsCompleted = 0
        self.firstPosition = [0, 0, 0]
        self.isFarfromfirst = False
        # self.minOccurcount = 3
        # self.coneRaduis = 1

    @mutexLock
    def saveOdometry(self, positionX: "float", positionY: "float", yaw: "float") -> None:
        """
        1-saves the first odometry message array in first position attribute to have a reference for
          distance measurnment and lap detection by determing if the current pose was moving away
          from the first position and then approaching it

        2-saves odometry message arrays in last position attribute iteritably
          to compute the needed calculations

        """
        if self.firstPosition is None:
            self.firstPosition = np.array([positionX, positionY, yaw])
        else:
            dist = np.linalg.norm(self.lastPosition[:2] - np.array([positionX, positionY])).astype(
                float
            )
            self.distTravelled += dist
            self.lastPosition = np.array([positionX, positionY, yaw])
            lastPose = self.lastPosition
            if self.firstPosition is not None:
                distTofirst = np.linalg.norm(lastPose[:2] - self.firstPosition[:2])
                if distTofirst > 10:
                    self.isFarfromfirst = True

                print(self.isFarfromfirst, distTofirst)
                if self.isFarfromfirst and distTofirst < 3:
                    self.isFarfromfirst = False
                    self.lapsCompleted += 1
                    print("HI")

    def parse(self, odomMsg: Odometry) -> typing.Tuple[float, float, float]:
        """
        this method parses the incoming odometry message to get
        quaternion info then tranforms it to euler

        """
        position = odomMsg.pose.pose.position
        quat = odomMsg.pose.pose.orientation
        positionX = quat.x
        positionY = quat.y
        positionZ = quat.z
        positionW = quat.w

        euler = euler_from_quaternion((positionX, positionY, positionZ, positionW))
        return position.x, position.y, euler[2]  # (x, y, yaw)

    @mutexLock
    def addCones(self, parsedCones: List[List[Any]]) -> None:
        """
        Transforms the given cones from the robot's local coordinate system
        to the global coordinate system,
        using the robot's last known position and orientation,
        and adds them to the list of unclustered cones.

        Args:
            parsedCones: A list of cones, where each cone is represented as a list with 3 elements:
                         - The X position of the cone in the robot's local coordinate system.
                         - The Y position of the cone in the robot's local coordinate system.
                         - The type of the cone.

        Returns:
            None.
        """
        poseX, poseY, yaw = self.lastPosition
        rotMat = np.eye(3)
        rotMat[0][0] = np.cos(yaw)
        rotMat[0][1] = -1 * np.sin(yaw)
        rotMat[1][0] = np.sin(yaw)
        rotMat[1][1] = np.cos(yaw)

        transformedCones: List[List[float]] = []
        for cone in parsedCones:
            coneX, coneY, coneType = cone

            # Transform point
            point = np.array([coneX, coneY, 1]).reshape(3, 1)
            transformed = rotMat @ point
            coneX = transformed[0][0] + poseX
            coneY = transformed[1][0] + poseY

            transformedCones.append([coneX, coneY, coneType])  # ask karim

        self.unclusteredCones.extend(transformedCones)

    def clusterCones(self) -> None:
        """
        Method Workflow:
        1. Check if there are any unclustered cones available for processing.
        2. If unclustered cones exist, create a new numpy array
        and copy the unclustered cones into it.
        3. Empty the `unclusteredCones` list.
        4. Create an empty numpy array `clusteredCones` to store clustered cones.
        5. If `clusteredCones` list already contains elements, convert it into a numpy array and
          keep only the x and y coordinates of the cones.
        6. Loop through each new cone in the `newCones` numpy array.
        7. Calculate the distance between the new cone
        and all the cones in the `clusteredCones` list.
        8. If a nearby cone exists, merge the new cone with the nearest cone
        by averaging their x and y coordinates
        and updating the cone type counts in the `clusteredColors` list.
        9. Otherwise, append the new cone to the `clusteredCones` list
        and initialize the cone type counts in the `clusteredColors` list.
        10. Convert `clusteredCones` list back to numpy array
        and keep only the x and y coordinates of the cones.

        """

        if len(self.unclusteredCones) > 0:
            newCones = np.copy(np.array(self.unclusteredCones)).astype(float)
            self.unclusteredCones = []
            clusteredCones: NDArray[Any] = np.array([])
            if len(self.clusteredCones) > 0:
                clusteredCones = np.array(self.clusteredCones)[:, :2].astype(float)

            for newCone in newCones:
                minDist = 9999
                minIdx: np.intp = np.intp(0)
                if clusteredCones.shape[0] > 0:
                    dists = np.linalg.norm(
                        np.subtract(newCone[:2].reshape(1, 2), clusteredCones), axis=1
                    )
                    minIdx = np.argmin(dists)
                    minDist = dists[minIdx]

                if minDist < 1:
                    coneCount = self.clusteredConescount[minIdx]
                    self.clusteredCones[minIdx] = (
                        clusteredCones[minIdx] * coneCount + newCone[:2]
                    ) / (coneCount + 1)
                    self.clusteredColors[minIdx][int(newCone[2])] += 1
                    clusteredCones = np.array(self.clusteredCones)[:, :2].astype(float)  # Very slow
                    self.clusteredConescount[minIdx] += 1
                else:
                    self.clusteredCones.append(newCone[:2])
                    self.clusteredConescount.append(1)
                    self.clusteredColors.append([0, 0, 0, 0, 0])
                    self.clusteredColors[minIdx][int(newCone[2])] += 1
                    clusteredCones = np.array(self.clusteredCones)[:, :2].astype(float)  # Very slow

    def getCones(self) -> typing.Tuple[NDArray[Any], NDArray[Any]]:
        """
        The getCones method takes the clustered cones and their colors,
        filters them based on a threshold, and returns the filtered cones and their colors.
        """
        self.clusterCones()
        if len(self.clusteredCones) == 0:
            return np.array([]), np.array([])

        counts = np.array(self.clusteredConescount)
        clusteredCones = np.array(self.clusteredCones)[:, :2].astype(float)[counts > 3]
        clusteredColors = np.array(self.clusteredColors)[counts > 3]
        probsColors: List["float"] = []
        for clusterC in clusteredColors:
            probsColors.append(clusterC[:3] / (np.sum(clusterC[:3] + 1)))

        if len(probsColors) == 0:
            bestColors: NDArray[Any] = np.array([])
        else:
            bestColors = np.argmax(probsColors, axis=1)
            bestColors[np.max(probsColors, axis=1) < 0.4] = 4

        return clusteredCones, bestColors
