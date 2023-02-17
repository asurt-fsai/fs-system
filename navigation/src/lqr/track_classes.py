"""
This module contains the dataclasses for the track.
"""
from dataclasses import dataclass
import numpy as np
import numpy.typing as npt


@dataclass
class Original:
    """
    This class contains the original track data.
    track: The track points
    alpha: The alpha values for the track points
    xCoeff: The x coefficients for the splines
    yCoeff: The y coefficients for the splines
    normVectors: The normal vectors of the track at each point
    noPoints: The number of points in the track
    """

    track: npt.NDArray[np.float64]
    alpha: npt.NDArray[np.float64] = np.array(None)
    xCoeff: npt.NDArray[np.float64] = np.array(None)

    yCoeff: npt.NDArray[np.float64] = np.array(None)
    normVectors: npt.NDArray[np.float64] = np.array(None)
    noPoints: int = 0

    def __post_init__(self) -> None:
        self.noPoints = self.track.shape[0]


@dataclass
class Smooth:
    """
    This class contains the smoothed track data.
    track: The track points
    alpha: The alpha values for the track points
    xCoeff: The x coefficients for the splines
    yCoeff: The y coefficients for the splines
    normVectors: The normal vectors of the track at each point
    noPoints: The number of points in the track
    noSplines: The number of splines in the track

    """

    track: npt.NDArray[np.float64] = np.array(None)
    alpha: npt.NDArray[np.float64] = np.array(None)
    xCoeff: npt.NDArray[np.float64] = np.array(None)
    yCoeff: npt.NDArray[np.float64] = np.array(None)
    normVectors: npt.NDArray[np.float64] = np.array(None)
    noPoints: int = 0
    noSplines: int = 0


@dataclass
class Optimized:
    """
    This class contains the optimized track data.
    track: The track points
    alpha: The alpha values for the track points
    normVectors: The normal vectors of the track at each point
    noPoints: The number of points in the track
    noSplines: The number of splines in the track
    """

    noPoints: int = 0
    noSplines: int = 0
    track: npt.NDArray[np.float64] = np.array(None)
    alpha: npt.NDArray[np.float64] = np.array(None)
    normVectors: npt.NDArray[np.float64] = np.array(None)


@dataclass
class Final:
    """
    This class contains the final track data.
    track: The track points
    alpha: The alpha values for the track points
    normVectors: The normal vectors of the track at each point
    noPoints: The number of points in the track
    noSplines: The number of splines in the track
    """

    noPoints: int
    noSplines: int
    track: npt.NDArray[np.float64]
    alpha: npt.NDArray[np.float64]
    normVectors: npt.NDArray[np.float64]
    xCoeff: npt.NDArray[np.float64]
    yCoeff: npt.NDArray[np.float64]


@dataclass
class Track:
    """
    This class conains all the track stages
    original: The original track data
    smooth: The smoothed track data
    optimized: The optimized track data
    final: The final track data
    """

    originial = Original
    smooth = Smooth
    optimized = Optimized
    final = Final
