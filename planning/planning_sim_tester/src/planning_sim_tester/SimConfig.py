"""
Module contains SimConfig to configure parameters for the simulator
and also contains PerformanceConfig to configure parameters for the performance estimator
"""
from dataclasses import dataclass


@dataclass
class SimConfig:
    """
    Class to hold configuration parameters for the simulator

    Parameters
    ----------
    coneNoise: float
        The standard deviation of the noise added to the cone positions
    coneDropProb: float
        The probability that a cone (run for each cone separately) will
        be dropped from an observation
    carNoise: float
        The standard deviation of the noise added to the car position
    visibility: float
        The maximum distance a cone can be seen from
    yellowColor: bool
        Whether the yellow cones have their color information added
    blueColor: bool
        Whether the blue cones have their color information added
    pWrongColor: float
        The probability that a cone will be given the wrong color
    colorConfBase: float
        The base confidence in the color of a cone
    colorConfNoise: float
        The standard deviation of the noise added to the color confidence
    successMaxDist: float
        The maximum distance from the goal that the car can be to be
        considered a success
    nSteps: int
        The number of steps to run the simulation for
    nExternalCones: int
        The number of external cones to add to the track
    externalConesNoise: float
        The standard deviation of the noise added to the external cone positions
    minExternalConeDist: float
        The minimum distance between an external cone and the track center line
    carVelocity: float
        The velocity of the car
    """

    coneNoise: float = 0.2
    coneDropProb: float = 0.3
    carNoise: float = 0.0001
    visibility: float = 7.0
    yellowColor: bool = False
    blueColor: bool = False
    pWrongColor: float = 0.00002
    colorConfBase: float = 0.8
    colorConfNoise: float = 0.1
    successMaxDist: float = 2
    nSteps: int = 1000
    nExternalCones: int = 1
    externalConesNoise: float = 20
    minExternalConeDist: float = 2.5
    carVelocity: float = 1.5
    blueConeType: int = 0
    yellowConeType: int = 1
    unknownConeType: int = 3
    timeStep: float = 0.5
