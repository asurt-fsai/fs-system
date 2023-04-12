"""
This module contains the function to create the raceline from the track's
reference line and the optimized alpha values, the generated raceline has
constant step size.
"""
from typing import Tuple
import math
import rospy
from tf2_geometry_msgs import PoseStamped
import numpy.typing as npt
import numpy as np
from nav_msgs.msg import Path
from .track import Track
from .lqrPrepareTrack import calcSplines

STEPSIZE_INTERP = rospy.get_param("/navigation/lqr/raceline/stepsize_interp")


def createRaceLine(
    track: Track,
) -> Tuple[Path, npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Given a track's reference line and the optimal alpha values, this function
    calculates the upper and lower bounds of the track as well as the
    optimal raceline as XY coordinates.
    ----------
    track: Track
        Track object

    Returns
    -------
    racelineMessage: Path
        Raceline as a path message
    interpRaceLine: np.ndarray, shape=(n, 2)
        Raceline as XY coordinates
    upperBound: np.ndarray, shape=(n, 2)
        Upper bound of the track as XY coordinates
    lowerBound: np.ndarray, shape=(n, 2)
        Lower bound of the track as XY coordinates
    """
    refline = track.smooth.track[:, :2]
    # calculate raceline on the basis of the optimized alpha values
    raceline = refline + np.expand_dims(track.optimized.alpha, 1) * track.smooth.normVectors

    # calculate new splines on the basis of the raceline
    closedRaceline = np.vstack((raceline, raceline[0]))

    (
        xCoeffRaceline,
        yCoeffRaceline,
        _,
        _,
        _,
    ) = calcSplines(path=closedRaceline)

    # calculate new spline lengths
    racelineSplineLengths = calcSplineLengths(xCoeff=xCoeffRaceline, yCoeff=yCoeffRaceline)

    # interpolate splines for evenly spaced raceline points
    interpRaceLine = interpSplines(
        splineLengths=racelineSplineLengths,
        xCoeff=xCoeffRaceline,
        yCoeff=yCoeffRaceline,
        stepSizeApprox=STEPSIZE_INTERP,
    )
    track.optimized.track = interpRaceLine
    upperBound, lowerBound = normVecsToTrackBound(track)
    racelineMessage = numpyToPath(interpRaceLine)
    return (racelineMessage, interpRaceLine, upperBound, lowerBound)


def calcSplineLengths(
    xCoeff: npt.NDArray[np.float64],
    yCoeff: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """
    This function calculates the length of each spline segment
    Parameters
    ----------
    xCoeff: np.ndarray, shape=(n, 4)
        x coefficients of the splines
    yCoeff: np.ndarray, shape=(n, 4)
        y coefficients of the splines
    Returns
    -------
    splineLengths: np.ndarray, shape=(n, 1)
        Length of each spline segment
    """
    # get number of splines and create output array
    noSplines = xCoeff.shape[0]
    splineLengths = np.zeros(noSplines)

    for i in range(noSplines):
        splineLengths[i] = math.sqrt(
            math.pow(np.sum(xCoeff[i]) - xCoeff[i, 0], 2)
            + math.pow(np.sum(yCoeff[i]) - yCoeff[i, 0], 2)
        )

    return splineLengths


def interpSplines(
    xCoeff: npt.NDArray[np.float64],
    yCoeff: npt.NDArray[np.float64],
    splineLengths: npt.NDArray[np.float64],
    stepSizeApprox: float,
) -> npt.NDArray[np.float64]:
    """
    This function interpolates the splines to create a raceline with constant
    step size.
    Parameters
    ----------
    xCoeff: np.ndarray, shape=(n, 4)
        x coefficients of the splines
    yCoeff: np.ndarray, shape=(n, 4)
        y coefficients of the splines
    splineLengths: np.ndarray, shape=(n, 1)
        Length of each spline segment
    stepSizeApprox: float
        Approximate step size of the raceline
    Returns
    -------
    interpRaceline: np.ndarray, shape=(n, 2)
        Interpolated raceline as XY coordinates
    """

    # calculate the cumulative spline lengths
    cumDist = np.cumsum(splineLengths)

    # calculate number of interpolation points and distances
    noInterpPoints = math.ceil(cumDist[-1] / stepSizeApprox) + 1
    interpDists = np.linspace(0.0, cumDist[-1], noInterpPoints)

    # create arrays to save the values
    interpRaceline = np.zeros((noInterpPoints, 2))  # raceline coords (x, y) array
    splineInds = np.zeros(noInterpPoints, dtype=int)
    tVals = np.zeros(noInterpPoints)

    # loop through all the elements and create steps with stepSizeApprox
    for i in range(noInterpPoints - 1):
        # find the spline that hosts the current interpolation point
        j = np.argmax(interpDists[i] < cumDist)
        splineInds[i] = j

        # get spline t value depending on the progress within the current element
        if j > 0:
            tVals[i] = (interpDists[i] - cumDist[j - 1]) / splineLengths[j]
        else:
            if splineLengths.ndim == 0:
                tVals[i] = interpDists[i] / splineLengths
            else:
                tVals[i] = interpDists[i] / splineLengths[0]

        # calculate coords
        interpRaceline[i, 0] = (
            xCoeff[j, 0]
            + xCoeff[j, 1] * tVals[i]
            + xCoeff[j, 2] * math.pow(tVals[i], 2)
            + xCoeff[j, 3] * math.pow(tVals[i], 3)
        )

        interpRaceline[i, 1] = (
            yCoeff[j, 0]
            + yCoeff[j, 1] * tVals[i]
            + yCoeff[j, 2] * math.pow(tVals[i], 2)
            + yCoeff[j, 3] * math.pow(tVals[i], 3)
        )

    interpRaceline[-1, 0] = np.sum(xCoeff[-1])
    interpRaceline[-1, 1] = np.sum(yCoeff[-1])
    splineInds[-1] = xCoeff.shape[0] - 1
    tVals[-1] = 1.0

    return interpRaceline


def normVecsToTrackBound(
    track: Track,
) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64],]:
    """
    Given a track's reference line and normal vectors this function
    calculates the upper and lower bounds of the track
    ----------
    track: Track
        Track class containing the track's reference line and normal vectors

    Returns
    -------
    trackUpBound: np.ndarray, shape=(n, 2)
        Upper bound of the track
    trackLowBound: np.ndarray, shape=(n, 2)
        Lower bound of the track
    """
    trackUpBound = track.smooth.track[:, :2] + track.smooth.normVectors * np.expand_dims(
        track.smooth.track[:, 2], 1
    )
    trackUpBound = np.vstack((trackUpBound, trackUpBound[0]))

    trackLowBound = track.smooth.track[:, :2] - track.smooth.normVectors * np.expand_dims(
        track.smooth.track[:, 3], 1
    )
    trackLowBound = np.vstack((trackLowBound, trackLowBound[0]))
    return (trackUpBound, trackLowBound)


def numpyToPath(pathArray: npt.NDArray[np.float64]) -> Path:
    """
    Converts a numpy array to a path message
    and transforms the coordinates to the world frame

    Parameters
    ----------
    pathArray: np.ndarray, shape=(n, 2)
        Numpy array with x and y coordinates of the path

    Returns
    -------
    path: Path
        Path message
    """
    print(len(pathArray[0]))
    path = Path()
    for i in range(len(pathArray[0])):
        pose = PoseStamped()
        pose.pose.position.x = pathArray[0][i]
        pose.pose.position.y = pathArray[1][i]
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        path.poses.append(pose)
    return path
