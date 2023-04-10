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
    noPoints: The number of points in the track
    distCum: The cumulative distance of each point of the input track
    noSplines: The number of splines in the track
    xCoeff: The x coefficients for the splines
    yCoeff: The y coefficients for the splines
    normVectors: The normal vectors of the track at each point
    """

    def __init__(self, refTrack: npt.NDArray[np.float64]):
        refTrack = np.vstack((refTrack, refTrack[0]))
        self.track = refTrack
        self.noPoints = self.track.shape[0]
        self.distCum = self.calcDistCum()
        self.noSplines: float
        self.alpha: npt.NDArray[np.float64]
        self.normVectors: npt.NDArray[np.float64]

    def calcDistCum(self) -> npt.NDArray[np.float64]:
        """
        Calculates the cumulative distance of each point of the input track

        Parameters
        ----------
        self

        Returns
        -------
        distsCumulative: np.array, shape=(N,1)
            Cumulative distances of the points.
        """
        distsCumulative: npt.NDArray[np.float64] = np.cumsum(
            np.sqrt(np.sum(np.power(np.diff(self.track[:, :2], axis=0), 2), axis=1))
        )
        distsCumulative = np.insert(distsCumulative, 0, 0.0)
        return distsCumulative


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
class SolverMatrices:
    """
    This class contains the helper matrices that
    will be used to calulate the solver input.

    Attributes
    ----------
    matP : npt.NDArray[np.float64]
        The P matrix, matP = [matPXX,matPXY,matPYY]
    matPrime : npt.NDArray[np.float64]
        The prime matrix, matPrime = [xPrime,yPrime]
    matT : npt.NDArray[np.float64]
        The T matrix, matT = [matTC,matTNX,matTNY]
    matQ : npt.NDArray[np.float64]
        The Q matrix, matQ = [matQX,matQY]
    curvPart : npt.NDArray[np.float64]
        The curvature part of the matrix
    """

    matP: npt.NDArray[np.float64] = np.array([None, None, None])
    matPrime: npt.NDArray[np.float64] = np.array([None, None])
    matT: npt.NDArray[np.float64] = np.array([None, None, None])
    matQ: npt.NDArray[np.float64] = np.array([None, None])
    curvPart: npt.NDArray[np.float64] = np.array(None)


@dataclass
class Track:
    """
    This class conains all the track stages
    original: The original track data
    smooth: The smoothed track data
    optimized: The optimized track data
    final: The final track data
    """

    def __init__(self, refTrack: npt.NDArray[np.float64]):
        self.original = Original(refTrack)
        self.smooth = Smooth()
        self.optimized = Optimized()