"""
Class to simulate running the planner on a track
It can be used to test the planner using multiple random configurations on the same track
"""
from typing import Callable, Any, List

import numpy as np
import numpy.typing as npt

from .trackgen import Track
from .SimConfig import SimConfig
from .ArcLengthSpline import ArcLengthSpline


class Simulator:
    """
    Class to simulate running the planner on a track
    """

    def __init__(self, simConfig: SimConfig, track: Track):
        self.simConfig = simConfig
        self.track = track

        arclengthSpline = ArcLengthSpline(numSamples=200)
        self.trackCenterLine = arclengthSpline.fitSpline(self.track.xMidline, self.track.yMidline)
        self.trackCenterPoint = np.mean(self.trackCenterLine, axis=0)

        self.yellowCones = np.array([self.track.xc1, self.track.yc1]).transpose()
        self.blueCones = np.array([self.track.xc2, self.track.yc2]).transpose()
        self.falseCones = np.empty(shape=(0, 2))

        self.carState = np.zeros(3)

        self.path: List[List[float]] = []
        self.losses: List[float] = []

    def getObservation(self, cones: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Observe the cones from the car's perspective

        Parameters
        ----------
        cones : npt.NDArray[np.float64]
            The positions of the cones

        Returns
        -------
        npt.NDArray[np.float64]
            The positions of the cones as seen by the car
        """
        carPos = self.carState[:2].reshape(1, 2)
        carAngle = self.carState[2]

        # Remove cones that are too far away
        distToCones = np.linalg.norm(cones - carPos, axis=1)
        filteredCones = cones[distToCones < self.simConfig.visibility]

        # Transform using the car's position
        filteredCones -= carPos

        # Transform using the car's angle
        rotMat = np.array(
            [[np.cos(carAngle), np.sin(carAngle)], [-np.sin(carAngle), np.cos(carAngle)]]
        )
        filteredCones = (rotMat @ filteredCones.transpose()).transpose()

        # Remove cones behind the car
        observedCones: npt.NDArray[np.float64] = filteredCones[filteredCones[:, 0] > 0]

        # to add: add view field/angle

        return observedCones

    def noiseDropCones(self, cones: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Add noise to the cone positions and drop some cones

        Parameters
        ----------
        cones : npt.NDArray[np.float64]
            The positions of the cones

        Returns
        -------
        npt.NDArray[np.float64]
            The noisy positions of the cones
        """
        cones += np.random.normal(0, self.simConfig.coneNoise, cones.shape)
        rands = np.random.uniform(0, 1, cones.shape[0])
        noisyCones: npt.NDArray[np.float64] = cones[rands > self.simConfig.coneDropProb]
        return noisyCones

    def getNoisyCones(self) -> npt.NDArray[np.float64]:
        """
        Get the noisy cones as seen by the car

        Returns
        -------
        npt.NDArray[np.float64]
            The noisy positions of the cones
            Each row is [x, y, coneType, coneColorConfidence]
        """
        # Get the cones as seen by the car
        yellowCones = self.getObservation(self.yellowCones)
        blueCones = self.getObservation(self.blueCones)
        falseCones = self.getObservation(self.falseCones)

        # Add noise and drop some cones
        noisyYellowCones = self.noiseDropCones(yellowCones)
        noisyBlueCones = self.noiseDropCones(blueCones)
        noisyFalseCones = self.noiseDropCones(falseCones)

        # Determine which colors are visible (otherwise the color type in unknown)
        blueConeType = self.simConfig.blueConeType
        yellowConeType = self.simConfig.yellowConeType
        if not self.simConfig.yellowColor:
            yellowConeType = self.simConfig.unknownConeType
        if not self.simConfig.blueColor:
            blueConeType = self.simConfig.unknownConeType

        finalColorYellow = []
        finalColorBlue = []
        for cone in noisyYellowCones:
            if np.random.uniform(0,1) < self.simConfig.pWrongColor:
                finalColorBlue.append(cone)
            else:
                finalColorYellow.append(cone)
            
        for cone in noisyBlueCones:
            if np.random.uniform(0,1) < self.simConfig.pWrongColor:
                finalColorYellow.append(cone)
            else:
                finalColorBlue.append(cone)

        # Add color type and color confidence to the cones
        noisyCones = []
        for cone in finalColorYellow:
            colorConfNoise = np.random.uniform(
                -self.simConfig.colorConfNoise, self.simConfig.colorConfNoise
            )
            noisyCones.append(
                [cone[0], cone[1], yellowConeType, self.simConfig.colorConfBase + colorConfNoise]
            )

        for cone in finalColorBlue:
            colorConfNoise = np.random.uniform(
                -self.simConfig.colorConfNoise, self.simConfig.colorConfNoise
            )
            noisyCones.append(
                [cone[0], cone[1], blueConeType, self.simConfig.colorConfBase + colorConfNoise]
            )

        for cone in noisyFalseCones:
            noisyCones.append([cone[0], cone[1], self.simConfig.unknownConeType, 0.5])

        return np.array(noisyCones)

    def generateFalseCones(self) -> None:
        """
        Generate false cones that are not on the track
        """

        falseConesX = np.random.normal(
            self.trackCenterPoint[0],
            self.simConfig.externalConesNoise,
            self.simConfig.nExternalCones,
        )
        falseConesY = np.random.normal(
            self.trackCenterPoint[1],
            self.simConfig.externalConesNoise,
            self.simConfig.nExternalCones,
        )
        falseCones = np.array([falseConesX, falseConesY]).transpose()

        # Remove cones that are too close to the center line
        distsToCenterLine = np.linalg.norm(
            falseCones.reshape((-1, 1, 2)) - self.trackCenterLine.reshape((1, -1, 2)), axis=2
        )
        closestDists = np.min(distsToCenterLine, axis=1)
        self.falseCones = falseCones[closestDists > self.simConfig.minExternalConeDist]

    def getWaypointDirection(self, waypoint: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Get the direction vector to the waypoint

        Parameters
        ----------
        waypoint : npt.NDArray[np.float64]
            A waypoint to move towards

        Returns
        -------
        npt.NDArray[np.float64]
            The direction vector to the next waypoint
        """
        carAngle = self.carState[2]
        rotMat = np.array(
            [[np.cos(carAngle), -np.sin(carAngle)], [np.sin(carAngle), np.cos(carAngle)]]
        )
        direction: npt.NDArray[np.float64] = (rotMat @ waypoint.reshape(2, 1)).reshape(-1)
        # print(waypoint, direction, carAngle)
        return direction

    def moveCar(self, nextWaypoint: npt.NDArray[np.float64], timeStep: float) -> None:
        """
        Moves the car towards the next waypoint

        Parameters
        ----------
        nextWaypoint : npt.NDArray[np.float64]
            The next waypoint to move towards
        timeStep: float
            The time step to move the car
        """
        direction = self.getWaypointDirection(nextWaypoint)
        directionNormal = direction / np.linalg.norm(direction)

        # Move car
        self.carState[:2] += self.simConfig.carVelocity * directionNormal * timeStep
        self.carState[2] = np.arctan2(direction[1], direction[0])

    def computeLoss(self, nextWaypoint: npt.NDArray[np.float64]) -> float:
        """
        Compute the loss for the next waypoint

        Parameters
        ----------
        nextWaypoint : npt.NDArray[np.float64]
            The next waypoint to move towards

        Returns
        -------
        float
            The loss is the closest distance to the track center line
        """
        carPos = self.carState[:2].reshape(1, 2)
        direction = self.getWaypointDirection(nextWaypoint).reshape(1, 2)
        globalWaypointPos = direction + carPos
        distsToTrackCenter = np.linalg.norm(globalWaypointPos - self.trackCenterLine, axis=1)
        closestDist: float = np.min(distsToTrackCenter)
        return closestDist

    def reset(self) -> None:
        """
        Resets the simulation
        """
        self.path = []
        self.losses = []

        self.falseCones = np.empty(shape=(0, 2))

        self.carState = np.zeros(3)

    def run(self, plannerClass: Callable[..., Any], carStartAngle: float = np.pi) -> bool:
        """
        Runs the simulation using a given planner

        Parameters
        ----------
        plannerClass : Callable
            Class of the planner to use
        carStartAngle : float, optional
            The starting angle of the car, by default np.pi

        Returns
        -------
        bool
            True if the car reached the end of the track, False otherwise
        """
        self.reset()
        self.carState = np.array([0, 0, carStartAngle])
        self.generateFalseCones()
        planner = plannerClass()

        for _ in range(self.simConfig.nSteps):
            self.path.append(np.copy(self.carState[:2]).tolist())

            # Get observations
            observedCones = self.getNoisyCones()

            # Run planning
            if len(observedCones) > 0:
                plannedPath = planner.getPath(observedCones)
                nextWaypoint = plannedPath[8]

            # Compute loss
            loss = self.computeLoss(nextWaypoint)
            self.losses.append(loss)
            if loss > self.simConfig.successMaxDist:
                return False

            # Move car and add noise
            self.moveCar(nextWaypoint, self.simConfig.timeStep)
            self.carState[:2] += np.random.normal(0, self.simConfig.carNoise, 2)

        return True
