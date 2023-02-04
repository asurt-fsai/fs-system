"""
Prepares the input track to be processed.
It consists of 3 steps:
    Linear Interpolation
    Smoothing
    B-Spline Interpolation
"""
import math
from typing import Tuple, List
from scipy import interpolate, optimize, spatial
import numpy.typing as npt
import numpy as np

K_REG: int = 3
S_REG: int = 10
STEPSIZE_PREP: float = 1.0
STEPSIZE_REG: float = 3.0
MIN_WIDTH: float = 3.0
TRACK_WIDTH: float = 3.0


class RefTrack:
    """
    Input Track given by Path Planning
    """

    def __init__(self, track: npt.NDArray[np.float64]):
        if track[-1] == track[0]:
            self.track = track
        else:
            track = np.vstack((track, track[0]))
            self.track = track
        self.noPoints = self.track.shape[0]
        self.distCum = self.calcDistCum()

    def addWidth(self) -> None:

        """
        Adds width of track to track array if only the x and y coordinates of the track are given

        Parameters
        ----------

        Returns
        -------
        """

        width = np.ones(self.noPoints) * TRACK_WIDTH
        if self.track.shape[1] == 2:
            self.track = np.hstack((self.track, width / 2, width / 2))

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


def prepTrack(
    refTrack: RefTrack,
) -> Tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    """
    Prepares the track

    Parameters
    ----------
    refTrackImp: np.array, shape=(M,4)
        Original track imported from SLAM [x,y,right width,left width]

    Return
    ------
    interpReferenceTrack: np.array, shape=(N,4)
        Interpolated track [x,y,right width,left width]
    normVecNormalizedInterp: np.array, shape=(N,2)
        Normalized normal vectors of each point in the interpolated track
    alphaInterp: np.array, shape=(N,4)
        Linear equation system coeffecients of interpolated track
    coeffXInterp
        Spline coeffecients of x component of interpolated track
    coeffYInterp
        Spline coeffecients of y component of interpolated track
    """
    # smoothing and interpolating reference track
    interpReferenceTrack = splineApprox(track=refTrack)

    # calculate splines
    refPathInterpClosed = np.vstack((interpReferenceTrack[:, :2], interpReferenceTrack[0, :2]))
    coeffXInterp, coeffYInterp, alphaInterp, normVecNormalizedInterp = calcSplines(
        path=refPathInterpClosed
    )

    for i in range(interpReferenceTrack.shape[0]):
        trackWidth = interpReferenceTrack[i, 2] + interpReferenceTrack[i, 3]

        if trackWidth < MIN_WIDTH:

            # inflate to both sidesTemp equally
            interpReferenceTrack[i, 2] += (MIN_WIDTH - trackWidth) / 2
            interpReferenceTrack[i, 3] += (MIN_WIDTH - trackWidth) / 2

    return (
        interpReferenceTrack,
        normVecNormalizedInterp,
        alphaInterp,
        coeffXInterp,
        coeffYInterp,
    )


def splineApprox(track: RefTrack) -> npt.NDArray[np.float64]:
    """
    Smooth spline approximation for track

    Parameters
    ----------
    track: np.array, shape=(M,4)
        array of all points and widths in track [x,y,right width, left width]

    Return
    ------
    trackSmooth: np.array, shape=(N,4)
        Smoothed track [x,y,right width,left width]
    """
    # Linear Interpolation before Smoothing
    trackInterp = interpTrack(track=track.track, stepsize=STEPSIZE_PREP)
    trackInterp = np.vstack((trackInterp, trackInterp[0]))

    # Spline Smoothing
    # find B spline representation of the inserted path and smooth it in this process
    # (pathSpline: tuple (vector of knots, the B-spline coefficients, and the degree of the spline))
    pathSpline, _ = interpolate.splprep(
        [trackInterp[:, 0], trackInterp[:, 1]], k=K_REG, s=S_REG, per=1
    )[:2]

    # calculate total length of smooth approximate spline based on euclidian distance at every 0.25m
    noPointsLenCalc = math.ceil(track.distCum[-1]) * 4
    smoothPath = np.array(
        interpolate.splev(np.linspace(0.0, 1.0, noPointsLenCalc), pathSpline)
    ).T  # Temp smooth path
    lenPathSmooth = np.sum(
        np.sqrt(np.sum(np.power(np.diff(smoothPath, axis=0), 2), axis=1))
    )  # Temp Lengths
    # get smoothed path
    noPointsSmooth = math.ceil(lenPathSmooth / STEPSIZE_REG) + 1
    smoothPath = np.array(interpolate.splev(np.linspace(0.0, 1.0, noPointsSmooth), pathSpline)).T[
        :-1
    ]

    # Calculate new track widths

    # find the closest points on the B spline to input points
    distsClosest = np.zeros(
        track.noPoints
    )  # contains (min) distances between input points and spline
    pointsClosest = np.zeros((track.noPoints, 2))  # contains the closest points on the spline
    tGlobClosest = np.zeros(track.noPoints)  # containts the tGlob values for closest points
    for i in range(track.noPoints):
        # get tGlob value for the point on the B spline with a minimum distance to the input points
        tGlobClosest[i] = optimize.fmin(
            distToP,
            x0=track.distCum[i] / track.distCum[-1],
            args=(pathSpline, track.track[i, :2]),
            disp=False,
        )

        # evaluate B spline on the basis of tGlob to obtain the closest point
        pointsClosest[i] = interpolate.splev(tGlobClosest[i], pathSpline)

        # save distance from closest point to input point
        distsClosest[i] = math.sqrt(
            math.pow(pointsClosest[i, 0] - track.track[i, 0], 2)
            + math.pow(pointsClosest[i, 1] - track.track[i, 1], 2)
        )

    # get side of smoothed track compared to the inserted track
    sides = np.zeros(track.noPoints - 1)

    for i in range(track.noPoints - 1):
        sides[i] = sideofLine(
            lineStart=track.track[i, :2], lineEnd=track.track[i + 1, :2], pointZ=pointsClosest[i]
        )

    sides = np.hstack((sides, sides[0]))

    # calculate new track widths on the basis of the new reference line
    # but not interpolated to new stepsize yet
    trackWidthRightInterp = track.track[:, 2] + sides * distsClosest
    trackWidthLeftInterp = track.track[:, 3] - sides * distsClosest

    # interpolate track widths after smoothing (linear)
    trackWidthRightInterp = np.interp(
        np.linspace(0.0, 1.0, noPointsSmooth), tGlobClosest, trackWidthRightInterp
    )
    trackWidthLeftInterp = np.interp(
        np.linspace(0.0, 1.0, noPointsSmooth), tGlobClosest, trackWidthLeftInterp
    )

    trackSmooth = np.column_stack(
        (smoothPath, trackWidthRightInterp[:-1], trackWidthLeftInterp[:-1])
    )

    return trackSmooth


def interpTrack(track: npt.NDArray[np.float64], stepsize: float) -> npt.NDArray[np.float64]:
    """
    Interpolate track points linearly to a new stepsize.

    Parameters
    ----------
    track: np.array, shape=(M,4)
        track in format [x,y,right width, left width]
    stepsize: float
        desired stepsize after interpolation in meters.

    Return
    ------
    interpTrack: np.array, shape=(N,4)
        interpolated track [x,y,right width,left width]
    """
    # create closed track
    # track = np.vstack((track, track[0]))
    # sum up total distance (from start) to every element
    distsCumulative = np.cumsum(np.sum(np.power(np.diff(track[:, :2], axis=0), 2), axis=1))
    distsCumulative = np.insert(distsCumulative, 0, 0.0)

    # calculate desired lengths using specified stepsize (+1 because last element is included)
    noPointsInterp = math.ceil(distsCumulative[-1] / stepsize) + 1
    distsInterp = np.linspace(0.0, distsCumulative[-1], noPointsInterp)

    # interpolate closed track points
    trackInterp = np.zeros((noPointsInterp, track.shape[1]))

    trackInterp[:, 0] = np.interp(distsInterp, distsCumulative, track[:, 0])
    trackInterp[:, 1] = np.interp(distsInterp, distsCumulative, track[:, 1])
    trackInterp[:, 2] = np.interp(distsInterp, distsCumulative, track[:, 2])
    trackInterp[:, 3] = np.interp(distsInterp, distsCumulative, track[:, 3])

    if track.shape[1] == 5:
        trackInterp[:, 4] = np.interp(distsInterp, distsCumulative, track[:, 4])

    return trackInterp[:-1]


def calcSplines(
    path: npt.NDArray[np.float64],
) -> Tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
]:
    """
    Solve for cubic splines (spline parameter t) between given points i
    (splines evaluated at t = 0 and t = 1).
    Ths splines are set up separately for x and y coordinates

    Parameters
    ----------
    path: np.array, shape=(M,2)
        x and y coordinates of the track

    Return
    ------
    xCoeffs: np.array, shape=(M-1,4)
        Spline coeffecients of x component
    yCoeffs: np.array, shape=(M-1,4)
        Spline coeffecients of y component
    splineAlpha: np.array, shape=(M-1,4)
        LES(Linear Equation System) coeffecients
    normVecNormal: np.array, shape=(M-1,2)
        normalized normal vectors of the input track at each point
    """
    # get number of splines
    noSplines = path.shape[0] - 1

    # alpha_{x,y} * a_{x,y} = b_{x,y}, a_{x,y} are the desired spline parameters
    # *4 because of 4 parameters in cubic spline
    splineAlpha = np.zeros((noSplines * 4, noSplines * 4))
    splineBX = np.zeros((noSplines * 4, 1))
    splineBY = np.zeros((noSplines * 4, 1))

    # create template for splineAlpha array entries
    # cubic spline s(t) = a_0i + a_1i*t + a_2i*t^2 + a_3i*t^3
    # row 1: beginning of current spline should be placed on current point (t = 0)
    # row 2: end of current spline should be placed on next point (t = 1)
    # row 3: heading at end of current spline should be equal to
    # heading at beginning of next spline (t = 1 and t = 0)
    # row 4: curvature at end of current spline should be equal to
    # curvature at beginning of next spline (t = 1 and t = 0)
    templateAlpha = np.array(
        [  # current point               | next point
            [1, 0, 0, 0, 0, 0, 0, 0],  # a_0i
            [1, 1, 1, 1, 0, 0, 0, 0],  # a_0i + a_1i +  a_2i +  a_3i
            [0, 1, 2, 3, 0, -1, 0, 0],  # _      a_1i + 2a_2i + 3a_3i      - a_1i+1
            [0, 0, 2, 6, 0, 0, -2, 0],  # _             2a_2i + 6a_3i               - 2a_2i+1
        ]
    )

    for i in range(noSplines):
        j = i * 4

        if i < noSplines - 1:
            splineAlpha[j : j + 4, j : j + 8] = templateAlpha

        else:
            # no curvature and heading bounds on last element
            splineAlpha[j : j + 2, j : j + 4] = [[1, 0, 0, 0], [1, 1, 1, 1]]

        splineBX[j : j + 2] = [[path[i, 0]], [path[i + 1, 0]]]
        splineBY[j : j + 2] = [[path[i, 1]], [path[i + 1, 1]]]

    # Set boundary conditions for first and last points
    # heading boundary condition
    splineAlpha[-2, 1] = 1
    splineAlpha[-2, -3:] = [-1, -2, -3]
    # curvature boundary condition
    splineAlpha[-1, 2] = 2
    splineAlpha[-1, -2:] = [-2, -6]

    # get coefficients of spline
    tSpline = np.arange(0, path.shape[0])

    xSpline = interpolate.CubicSpline(tSpline, path[:, 0])
    ySpline = interpolate.CubicSpline(tSpline, path[:, 1])

    xCoeffs = np.rot90(xSpline.c, 3)
    yCoeffs = np.rot90(ySpline.c, 3)

    # get normal vector
    normVec = np.stack((yCoeffs[:, 1], -xCoeffs[:, 1]), axis=1)

    # normalize normal vectors
    normVecNormal = (
        np.expand_dims(1.0 / np.sqrt(np.sum(np.power(normVec, 2), axis=1)), axis=1) * normVec
    )
    return xCoeffs, yCoeffs, splineAlpha, normVecNormal


def distToP(
    tGlob: npt.NDArray[np.float64], path: List[float], pathPoint: npt.NDArray[np.float64]
) -> float:
    """
    Calculate the distance from point on path to a point on the spline at spline parameter tGlob

    Parameters
    ----------
    tGlob: np.array, shape=(N,1)
        spline parameter to calculate the distance at
    path: np.array, shape=(M,2)
        x and y coordinates of the track
    pathPoint: np.array, shape=(1,2)

    Return
    ------
    distance: float
        distance from point pathPoint to points on the spline at tGlob
    """
    splinePoint = np.array(interpolate.splev(tGlob, path))
    splinePoint = splinePoint.reshape(-1)
    pathPoint = pathPoint.reshape(-1)
    distance: float = spatial.distance.euclidean(pathPoint, splinePoint)
    return distance


def sideofLine(
    lineStart: npt.NDArray[np.float64],
    lineEnd: npt.NDArray[np.float64],
    pointZ: npt.NDArray[np.float64],
) -> float:
    """
    Determine if point z is to the right or left of a line from a to b

    Parameters
    ----------
    lineStart: np.array, shape=(1,2)
        Start point of line ab
    lineEnd: np.array, shape=(1,2)
        End point of line ab
    pointZ: np.array, shape=(1,2)
        Point to find the side of

    Return
    ------
    side: float
        Side of point z, (0.0 = on line, 1.0 = left side, -1.0 = right side)

    """
    side: float = np.sign(
        (lineEnd[0] - lineStart[0]) * (pointZ[1] - lineStart[1])
        - (lineEnd[1] - lineStart[1]) * (pointZ[0] - lineStart[0])
    )
    return side
