"""
Module containign ArcLengthSpline class used to fit a parametric third order
spline as a function of arclength
"""
from typing import Tuple, Optional
import numpy.typing as npt

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


def findInterval(xPoints: npt.NDArray[np.float64], x: float) -> int:
    """
    Finds the interval in which x is located

    Note: this can be improved by using binary search

    Parameters
    ----------
    xPoints : npt.NDArray[np.float64]
        List of x values of points
    x : float
        Value to find interval for

    Returns
    -------
    int
        Index of interval in which x is located
    """
    idx = 0
    for idx, xValue in enumerate(xPoints):
        if xValue > x:
            break
    return idx - 1


class SplineInterpolator:
    """
    This class fits a parametric third order spline

    Parameters
    ----------
    xPoints: npt.NDArray[np.float64]
        List of x values for points spline is fitted on
    yPoints: npt.NDArray[np.float64]
        List of y values for points spline is fitted on
    """

    def __init__(self, xPoints: npt.NDArray[np.float64], yPoints: npt.NDArray[np.float64]):
        self.xPoints = xPoints
        self.yPoints = yPoints
        self.spline = CubicSpline(xPoints, yPoints, bc_type="natural")
        self.coeffs = self.spline.c

    def evaluate(self, x: float) -> float:
        """
        Evaluate the spline at a given x value

        Parameters
        ----------
        x: float
            The x value to evaluate the spline at

        Returns
        -------
        float
            The y value of the spline at the given x value
        """
        interval = findInterval(self.xPoints, x)

        if interval < 0 or interval > self.coeffs.shape[1]:
            print(f"WARNING: Evaluating spline outside its bounds at value {x}")

        interval = max(interval, 0)

        ans = 0
        x -= self.xPoints[interval]

        for idx, coeffs in enumerate(self.coeffs[:, interval][::-1]):
            ans += coeffs * (x**idx)

        return ans


class ArcLengthSpline:
    """
    This class fits a parametric third order spline as a function of arclength
    Note: it also resamples the given points in order to have equal
    distances (euclidean) between each two knots

    Parameters
    ----------
    numSamples: int
        The number of knots create when resampling
    arclengthDt: float
        The delta time used when estimated the arclength
    """

    def __init__(self, numSamples: int = 50, arclengthDt: float = 0.01):
        self.arclengthDt = arclengthDt
        self.numSamples = numSamples

        self.tPoints = np.zeros(0)
        self.xSpline: Optional[SplineInterpolator] = None
        self.ySpline: Optional[SplineInterpolator] = None

        self.totalArclength = 0.0
        self.deltaArclength = 0.0

    def fitSpline(
        self, xPoints: npt.NDArray[np.float64], yPoints: npt.NDArray[np.float64], plot: bool = False
    ) -> npt.NDArray[np.float64]:
        """
        Fits a parametric third order spline as a function of arclength

        Parameters
        ----------
        xPoints : npt.NDArray[np.float64]
            The x values of the points to fit the spline on
        yPoints : npt.NDArray[np.float64]
            The y values of the points to fit the spline on
        plot : bool, optional, by default False
            Whether to plot the resampled points

        Returns
        -------
        npt.NDArray[np.float64]
            The resampled points
        """
        # Fit initial spline objects
        self.tPoints = np.arange(0, xPoints.shape[0]).astype(float)
        self.xSpline = SplineInterpolator(self.tPoints, xPoints)
        self.ySpline = SplineInterpolator(self.tPoints, yPoints)

        # Get arclengths of the whole spline
        arclengths, timeVals = self.computeTotalArclength()
        totalArclength = np.sum(arclengths).item()
        self.totalArclength = totalArclength

        self.deltaArclength = totalArclength / self.numSamples

        # Resample using self.deltaArclength between each two points
        newTimeVals = [0]  # Will have arclength from start to each point
        currentTheta = self.deltaArclength
        newXPoints = [self.xSpline.evaluate(0)]
        newYPoints = [self.ySpline.evaluate(0)]

        for idx in range(arclengths.shape[0]):
            if arclengths[:idx].sum() >= currentTheta or idx == arclengths.shape[0] - 1:
                currentTheta += self.deltaArclength
                newTimeVal = timeVals[idx - 1]
                newTimeVals.append(arclengths[:idx].sum())
                newXPoints.append(self.xSpline.evaluate(newTimeVal))
                newYPoints.append(self.ySpline.evaluate(newTimeVal))

        newXPointsArr = np.array(newXPoints)
        newYPointsArr = np.array(newYPoints)

        # Create new spline objects to fit resampled points
        self.tPoints = np.array(newTimeVals)
        self.xSpline = SplineInterpolator(self.tPoints, newXPointsArr)
        self.ySpline = SplineInterpolator(self.tPoints, newYPointsArr)

        # Get resampled points from the fitted splines
        points = self.resamplePointsFromSpline()

        if plot:
            plt.title("Resampled points")
            plt.scatter(points[:, 0], points[:, 1])
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.show()
        return points

    def resamplePointsFromSpline(self) -> npt.NDArray[np.float64]:
        """
        Resamples points from the fitted arclength based spline

        Returns
        -------
        npt.NDArray[np.float64]
            The resampled points

        Raises
        ------
        ValueError
            If the spline has not been fitted yet
        """
        if self.xSpline is None or self.ySpline is None:
            raise ValueError("Spline not fitted yet")
        points = []
        for i in range(self.numSamples):
            theta = self.deltaArclength * i
            x, y = self.xSpline.evaluate(theta), self.ySpline.evaluate(theta)
            points.append([x, y])
        return np.array(points)

    def evaluate(self, theta: float) -> Tuple[float, float]:
        """
        Evaluates the spline at a given arclength

        Parameters
        ----------
        theta : float
            The arclength to evaluate the spline at

        Returns
        -------
        Tuple[float, float]
            The x and y values of the spline at the given arclength

        Raises
        ------
        ValueError
            If the spline has not been fitted yet
        """
        if self.xSpline is None or self.ySpline is None:
            raise ValueError("Spline not fitted yet")
        interval = int(theta // self.deltaArclength)
        ansX, ansY = 0, 0
        xCoeffs = self.xSpline.coeffs[:, interval][::-1]
        yCoeffs = self.ySpline.coeffs[:, interval][::-1]
        dTheta = theta - interval * self.deltaArclength

        for idx in range(xCoeffs.shape[0]):
            polyTerm = dTheta**idx
            ansX += xCoeffs[idx] * polyTerm
            ansY += yCoeffs[idx] * polyTerm

        return ansX, ansY

    def estimateIntervalArclength(
        self, interval: int
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Estimates the arclength of one cubic parameteric curve from the splines

        Parameters
        ----------
        interval: int
            The interval of the cubic parameteric curve to estimate the arclength of

        Returns
        -------
        lengths: npt.NDArray[np.float64]
            Arclengths between points on the given parameteric curve that are self.arclengthDt apart
        timeVals: npt.NDArray[np.float64]
            The values of t on points on the given parameteric curve that are self.arclengthDt apart

        Raises
        ------
        ValueError
            If the spline has not been fitted yet
        """
        if self.xSpline is None or self.ySpline is None:
            raise ValueError("Spline not fitted yet")

        xCoeffs = self.xSpline.coeffs[:, interval][::-1]
        yCoeffs = self.ySpline.coeffs[:, interval][::-1]

        timeVals = np.arange(interval, interval + 1 + self.arclengthDt, self.arclengthDt) - interval

        # Evalute the splines at points timeVals
        pointsX = np.zeros(timeVals.shape[0])
        pointsY = np.zeros(timeVals.shape[0])
        for idx in range(xCoeffs.shape[0]):
            pointsX += xCoeffs[idx] * (timeVals**idx)
            pointsY += yCoeffs[idx] * (timeVals**idx)

        # Calculate the distance between each 2 consecutive points
        xDiffs = pointsX[:-1] - pointsX[1:]
        yDiffs = pointsY[:-1] - pointsY[1:]
        xDiffs *= xDiffs
        yDiffs *= yDiffs
        distSums = xDiffs + yDiffs
        lengths = np.sqrt(distSums)

        timeVals = timeVals[:-1] + interval

        return lengths, timeVals

    def computeTotalArclength(self) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Estimate the arclength of the whole spline (all of the cubic polynomials fitted)

        Returns
        -------
        arclengths: npt.NDArray[np.float64]
            Arclengths between points on the fitted spline that are self.arclengthDt apart
        timeVals: npt.NDArray[np.float64]
            The values of t on points on the fitted spline that are self.arclengthDt apart
        """
        arclengths = []
        timeVals = []
        for i in range(self.tPoints.shape[0] - 1):
            intervalArclengths, intervalTimeVals = self.estimateIntervalArclength(i)

            arclengths.extend(intervalArclengths.tolist())
            timeVals.extend(intervalTimeVals.tolist())

        return np.array(arclengths), np.array(timeVals)
