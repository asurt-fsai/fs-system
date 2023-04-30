#!/usr/bin/env python
"""
Prepares the input track to be processed.
It consists of 3 steps:
    Linear Interpolation
    Smoothing
    B-Spline Interpolation
"""
import math
from typing import Tuple, List
from dataclasses import dataclass
from scipy import interpolate, optimize, spatial
import rospy
import numpy.typing as npt
import numpy as np

K_REG: int = rospy.get_param("/navigation/lqr/prepare_track/k_reg")
S_REG: int = rospy.get_param("/navigation/lqr/prepare_track/s_reg")
STEPSIZE_PREP: float = rospy.get_param("/navigation/lqr/prepare_track/stepsize_prep")
STEPSIZE_REG: float = rospy.get_param("/navigation/lqr/prepare_track/stepsize_reg")
TRACK_WIDTH: float = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN: float = rospy.get_param("/navigation/lqr/handler/safety_margin")


@dataclass
class SmoothTrackCoeffs:
    """
    This class contains the smooth track coeffecients and the normal vectors.
    xCoeff: The x-coefficients of the track spline
    yCoeff: The y-coefficients of the track spline
    alpha: The alpha values of the smooth track
    normVectors: The normal vectors of the smooth path
    """

    xCoeff: npt.NDArray[np.float64] = np.array(None)
    yCoeff: npt.NDArray[np.float64] = np.array(None)
    alpha: npt.NDArray[np.float64] = np.array(None)
    normVectors: npt.NDArray[np.float64] = np.array(None)


class SmoothTrack:
    """
    This class contains the smoothed track data.
    refDistCum: The cumulative distance along the original ref path.
    path: Coordinate points of the + right and left widths.
    alpha: The alpha values for the track points
    xCoeff: The x coefficients for the splines
    yCoeff: The y coefficients for the splines
    normVectors: The normal vectors of the track at each point
    noPoints: The number of points in the track
    noSplines: The number of splines in the track
    """

    def __init__(self, originalPath: npt.NDArray[np.float64]) -> None:
        self.refDistCum: npt.NDArray[np.float64] = self.calcDistCum(originalPath)
        self.path: npt.NDArray[np.float64] = np.array(None)
        self.trackCoeffs: SmoothTrackCoeffs = SmoothTrackCoeffs()
        self.noPoints: int = 0

        self.originalToSmooth(originalPath)

    def originalToSmooth(
        self,
        refTrack: npt.NDArray[np.float64],
    ) -> None:
        """
        Prepares the track

        Parameters
        ----------
        refTrack: originalPath
            The original input track.

        Return
        ------
        None
        """
        # smoothing and interpolating reference track
        self.path = self.splineApprox(track=refTrack)

        # calculate splines
        refPathInterpClosed = np.vstack((self.path[:, :2], self.path[0, :2]))
        (
            self.trackCoeffs.xCoeff,
            self.trackCoeffs.yCoeff,
            self.trackCoeffs.alpha,
            self.trackCoeffs.normVectors,
        ) = self.calcSplines(path=refPathInterpClosed)

        self.noPoints = self.path.shape[0]
        self.noSplines = self.noPoints

    def splineApprox(self, track: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Smooth spline approximation for track

        Parameters
        ----------
        track: np.array, shape=(M,4)
            array of all points and widths in track [x,y,right width, left width]

        Return
        ------
        np.array, shape=(N,4)
            Smoothed track [x,y,right width,left width]
        """
        # Linear Interpolation before Smoothing
        trackInterp = self.interpTrack(track, stepsize=STEPSIZE_PREP)
        trackInterp = np.vstack((trackInterp, trackInterp[0]))

        # Spline Smoothing
        # find B spline representation of the inserted path and smooth it in this process
        # (pathSpline: (vector of knots, the B-spline coefficients, and the degree of the spline))
        pathSpline, _ = interpolate.splprep(
            [trackInterp[:, 0], trackInterp[:, 1]], k=K_REG, s=S_REG, per=1
        )[:2]

        # calculate total length of smooth approximate spline
        # based on euclidian distance at every 0.25m
        noPointsLenCalc = math.ceil(self.refDistCum[-1]) * 4
        smoothPath = np.array(
            interpolate.splev(np.linspace(0.0, 1.0, noPointsLenCalc), pathSpline)
        ).T  # Temp smooth path
        lenPathSmooth = np.sum(
            np.sqrt(np.sum(np.power(np.diff(smoothPath, axis=0), 2), axis=1))
        )  # Temp Lengths
        # get smoothed path
        noPointsSmooth = math.ceil(lenPathSmooth / STEPSIZE_REG) + 1
        smoothPath = np.array(
            interpolate.splev(np.linspace(0.0, 1.0, noPointsSmooth), pathSpline)
        ).T[:-1]

        # Calculate new track widths

        # find the closest points on the B spline to input points
        distsClosest = np.zeros(
            track.shape[0]
        )  # contains (min) distances between input points and spline
        pointsClosest = np.zeros((track.shape[0], 2))  # contains the closest points on the spline
        tGlobClosest = np.zeros(track.shape[0])  # containts the tGlob values for closest points
        for i in range(track.shape[0]):
            # get tGlob value for the point on the B spline
            # with a minimum distance to the input points
            tGlobClosest[i] = optimize.fmin(
                self.distToP,
                x0=self.refDistCum[i] / self.refDistCum[-1],
                args=(pathSpline, track[i, :2]),
                disp=False,
            )

            # evaluate B spline on the basis of tGlob to obtain the closest point
            pointsClosest[i] = interpolate.splev(tGlobClosest[i], pathSpline)

            # save distance from closest point to input point
            distsClosest[i] = math.sqrt(
                math.pow(pointsClosest[i, 0] - track[i, 0], 2)
                + math.pow(pointsClosest[i, 1] - track[i, 1], 2)
            )

        # get side of smoothed track compared to the inserted track
        sides = np.zeros(track.shape[0] - 1)

        for i in range(track.shape[0] - 1):
            sides[i] = self.sideofLine(
                lineStart=track[i, :2],
                lineEnd=track[i + 1, :2],
                pointZ=pointsClosest[i],
            )

        sides = np.hstack((sides, sides[0]))

        # calculate new track widths on the basis of the new reference line
        # but not interpolated to new stepsize yet
        trackWidthRightInterp = track[:, 2] + sides * distsClosest
        trackWidthLeftInterp = track[:, 3] - sides * distsClosest

        # interpolate track widths after smoothing (linear)
        trackWidthRightInterp = np.interp(
            np.linspace(0.0, 1.0, noPointsSmooth), tGlobClosest, trackWidthRightInterp
        )
        trackWidthLeftInterp = np.interp(
            np.linspace(0.0, 1.0, noPointsSmooth), tGlobClosest, trackWidthLeftInterp
        )

        return np.column_stack((smoothPath, trackWidthRightInterp[:-1], trackWidthLeftInterp[:-1]))

    def interpTrack(
        self, originalPath: npt.NDArray[np.float64], stepsize: float
    ) -> npt.NDArray[np.float64]:
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
        # calculate desired lengths using specified stepsize (+1 because last element is included)
        noPointsInterp = math.ceil(self.refDistCum[-1] / stepsize) + 1
        distsInterp = np.linspace(0.0, self.refDistCum[-1], noPointsInterp)

        # interpolate closed track points
        trackInterp = np.zeros((noPointsInterp, originalPath.shape[1]))

        trackInterp[:, 0] = np.interp(distsInterp, self.refDistCum, originalPath[:, 0])
        trackInterp[:, 1] = np.interp(distsInterp, self.refDistCum, originalPath[:, 1])
        trackInterp[:, 2] = np.interp(distsInterp, self.refDistCum, originalPath[:, 2])
        trackInterp[:, 3] = np.interp(distsInterp, self.refDistCum, originalPath[:, 3])

        if originalPath.shape[1] == 5:
            trackInterp[:, 4] = np.interp(distsInterp, self.refDistCum, originalPath[:, 4])

        return trackInterp[:-1]

    def calcSplines(
        self,
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
        noSplines: int = path.shape[0] - 1

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
        self.trackCoeffs.normVectors = (
            np.expand_dims(1.0 / np.sqrt(np.sum(np.power(normVec, 2), axis=1)), axis=1) * normVec
        )
        return xCoeffs, yCoeffs, splineAlpha, self.trackCoeffs.normVectors

    def distToP(
        self, tGlob: npt.NDArray[np.float64], path: List[float], pathPoint: npt.NDArray[np.float64]
    ) -> float:
        """
        Calculate the distance from point on path to a point on the spline at spline parameter tGlob

        Parameters
        ----------
        tGlob: np.array, shape=(N,1)
            spline parameter to calculate the distance at
        path: list, shape=(M,1)
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
        self,
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

    def calcDistCum(self, path: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Calculates the cumulative distance of each point of the input path

        Parameters
        ----------
        self

        Returns
        -------
        distsCumulative: np.array, shape=(N,1)
            Cumulative distances of the points.
        """
        distsCumulative: npt.NDArray[np.float64] = np.cumsum(
            np.sqrt(np.sum(np.power(np.diff(path[:, :2], axis=0), 2), axis=1))
        )
        distsCumulative = np.insert(distsCumulative, 0, 0.0)
        return distsCumulative

    if __name__ == "__main__":
        pass
