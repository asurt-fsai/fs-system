"""
This module contains the function to create the raceline from the track's
reference line and the optimized alpha values, the generated raceline has
constant step size.
"""
from typing import Tuple
import math
from dataclasses import dataclass
import rospy
import quadprog
from tf2_geometry_msgs import PoseStamped
from scipy import interpolate
import numpy.typing as npt
import numpy as np
from nav_msgs.msg import Path
from lqr import SmoothTrack, SolverMatrices


STEPSIZE_INTERP = rospy.get_param("/navigation/lqr/raceline/stepsize_interp")
VEHICLE_WIDTH: float = rospy.get_param("/navigation/lqr/optimize_track/vehicle_width")
CURVATURE_BOUNDARIES: float = rospy.get_param("/navigation/lqr/optimize_track/curvature_boundaries")


@dataclass
class TrackCoeffs:
    """
    This class contains the track coeffecients and the normal vectors.
    xCoeff: The x-coefficients of the track
    yCoeff: The y-coefficients of the track
    alpha: The alpha values of the track
    normVectors: The normal vectors of the track
    """

    xCoeff: npt.NDArray[np.float64] = np.array(None)
    yCoeff: npt.NDArray[np.float64] = np.array(None)
    alpha: npt.NDArray[np.float64] = np.array(None)
    normVectors: npt.NDArray[np.float64] = np.array(None)


@dataclass
class TrackBounds:
    """
    This class contains the upper and lower bounds for the track.
    upperBound: The upper bound for the track
    lowerBound: The lower bound for the track
    """

    upperBound: npt.NDArray[np.float64] = np.array(None)
    lowerBound: npt.NDArray[np.float64] = np.array(None)
    upperBoundMsg: Path = Path()
    lowerBoundMsg: Path = Path()


class OptimizedTrack:
    """
    This class contains the optimized track data.
    interpRaceline: The interpolated raceline
    trackCoeffs: The track coeffecients
    bounds: The track bounds
    smoothTrack: The smooth track
    solverMat: The solver matrices

    """

    def __init__(self, smoothTrack: SmoothTrack):
        self.interpRaceline: npt.NDArray[np.float64] = np.array(None)
        self.trackCoeffs: TrackCoeffs = TrackCoeffs()
        self.bounds: TrackBounds = TrackBounds()
        self.smoothTrack: SmoothTrack = smoothTrack
        self.solverMat: SolverMatrices = SolverMatrices(self.smoothTrack)
        self.optimizeMinCurve()
        self.createRaceLine()

    def optimizeMinCurve(self) -> npt.NDArray[np.float64]:
        """
        This function optimizes the minimum curvature of the track.
        It uses the scipy.optimize.minimize function to minimize the cost function.
        The cost function is the sum of the quadratic cost matrices.
        The function returns the optimized track coordinates.

        Parameters
        ----------
        self

        Returns
        -------
        alphaMinCurve : npt.NDArray[np.float64]
            Array containing the optimized track alphas
            which are going to be multiplied by the normal vector
            to get the raceline coordinates
        """
        tempCostMat = np.array([None, None, None])
        tempCostMat[0] = np.matmul(
            self.solverMat.matT[1].T, np.matmul(self.solverMat.matP[0], self.solverMat.matT[1])
        )
        tempCostMat[1] = np.matmul(
            self.solverMat.matT[2].T, np.matmul(self.solverMat.matP[1], self.solverMat.matT[1])
        )
        tempCostMat[2] = np.matmul(
            self.solverMat.matT[2].T, np.matmul(self.solverMat.matP[2], self.solverMat.matT[2])
        )
        costMatQuad = tempCostMat[0] + tempCostMat[1] + tempCostMat[2]
        # make costMatQuad symmetric(because solver used needs symmetrical)

        costMatQuad = (costMatQuad + costMatQuad.T) / 2
        tempCostMat = np.array([None, None, None])
        tempCostMat[0] = 2 * np.matmul(
            np.matmul(self.solverMat.matQ[0].T, self.solverMat.matT[0].T),
            np.matmul(self.solverMat.matP[0], self.solverMat.matT[1]),
        )
        tempCostMat[1] = np.matmul(
            np.matmul(self.solverMat.matQ[0].T, self.solverMat.matT[0].T),
            np.matmul(self.solverMat.matP[1], self.solverMat.matT[2]),
        ) + np.matmul(
            np.matmul(self.solverMat.matQ[1].T, self.solverMat.matT[0].T),
            np.matmul(self.solverMat.matP[1], self.solverMat.matT[1]),
        )
        tempCostMat[2] = 2 * np.matmul(
            np.matmul(self.solverMat.matQ[1].T, self.solverMat.matT[0].T),
            np.matmul(self.solverMat.matP[2], self.solverMat.matT[2]),
        )
        costMat = tempCostMat[0] + tempCostMat[1] + tempCostMat[2]
        costMat = np.squeeze(costMat)  # remove non-singleton dimensions

        # CURVATURE(KAPPA) CONSTRAINTS
        matCurvCons = np.array([None, None])
        matCurvCons[0] = np.matmul(self.solverMat.curvPart, self.solverMat.matPrime[1])
        matCurvCons[1] = np.matmul(self.solverMat.curvPart, self.solverMat.matPrime[0])

        print("\n Q_y: \n", matCurvCons[1], "\n")
        print("\n T_ny: \n", self.solverMat.matT[2], "\n")
        print("\n Q_x: \n", matCurvCons[0], "\n")
        print("\n T_nx: \n", self.solverMat.matT[1], "\n")

        # this part is multiplied by alpha within the optimization
        curvature = np.matmul(matCurvCons[1], self.solverMat.matT[2]) - np.matmul(
            matCurvCons[0], self.solverMat.matT[1]
        )
        # print("\n Curvature: \n", curvature)
        # original curvature part (static part)
        curvReference = np.matmul(
            matCurvCons[1], np.matmul(self.solverMat.matT[0], self.solverMat.matQ[1])
        )
        curvReference -= np.matmul(
            matCurvCons[0], np.matmul(self.solverMat.matT[0], self.solverMat.matQ[0])
        )

        upperCon = np.ones((self.smoothTrack.noPoints, 1)) * CURVATURE_BOUNDARIES - curvReference
        lowerCon = -(
            np.ones((self.smoothTrack.noPoints, 1)) * (-1 * CURVATURE_BOUNDARIES) - curvReference
        )
        # Solve a Quadratic Program defined as:
        #    minimize
        #        (1/2) * alpha.T * costMatQuad * alpha + costMat.T * alpha
        #    subject to
        #        constCoeff * alpha <= constrains

        # calculate allowed deviation from refline
        maxDevRight = self.smoothTrack.path[:, 2] - (VEHICLE_WIDTH / 2)
        maxDevLeft = self.smoothTrack.path[:, 3] - (VEHICLE_WIDTH / 2)

        # consider value boundaries (-maxDevLeft <= alpha <= maxDevRight)
        constCoeff = np.vstack(
            (
                np.eye(self.smoothTrack.noPoints),
                -np.eye(self.smoothTrack.noPoints),
                curvature,
                -curvature,
            )
        )
        constrains = np.append(maxDevRight, maxDevLeft)
        constrains = np.append(constrains, upperCon)
        constrains = np.append(constrains, lowerCon)

        # print(constCoeff)
        # solve problem
        alphaMinCurve: npt.NDArray[np.float64] = quadprog.solve_qp(
            costMatQuad, -costMat, -constCoeff.T, -constrains, 0
        )[0]
        self.trackCoeffs.yCoeff = alphaMinCurve
        return alphaMinCurve

    def createRaceLine(
        self,
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
        refline = self.smoothTrack.path[:, :2]
        # calculate raceline on the basis of the optimized alpha values
        print("refline shape: ", refline.shape)
        print("alpha shape: ", self.trackCoeffs.yCoeff.shape)
        print("normVectors shape: ", self.smoothTrack.trackCoeffs.normVectors.shape)
        raceline = (
            refline
            + np.expand_dims(self.trackCoeffs.yCoeff, 1) * self.smoothTrack.trackCoeffs.normVectors
        )

        # calculate new splines on the basis of the raceline
        closedRaceline = np.vstack((raceline, raceline[0]))

        (
            self.trackCoeffs.xCoeff,
            self.trackCoeffs.yCoeff,
        ) = self.calcSplines(closedRaceline)

        # calculate new spline lengths
        racelineSplineLengths = self.calcSplineLengths(
            xCoeff=self.trackCoeffs.xCoeff, yCoeff=self.trackCoeffs.yCoeff
        )

        # interpolate splines for evenly spaced raceline points
        self.interpRaceline = self.interpSplines(
            splineLengths=racelineSplineLengths,
            xCoeff=self.trackCoeffs.xCoeff,
            yCoeff=self.trackCoeffs.yCoeff,
            stepSizeApprox=STEPSIZE_INTERP,
        )
        self.bounds.upperBound, self.bounds.lowerBound = self.normVecsToTrackBound()
        print("interpRaceLine: ", self.interpRaceline.shape)
        self.racelineMsg = self.numpyToPath(self.interpRaceline)
        self.bounds.upperBoundMsg = self.numpyToPath(self.bounds.upperBound)
        self.bounds.lowerBoundMsg = self.numpyToPath(self.bounds.lowerBound)
        return (
            self.racelineMsg,
            self.interpRaceline,
            self.bounds.upperBound,
            self.bounds.lowerBound,
        )

    def calcSplineLengths(
        self,
        xCoeff: npt.NDArray[np.float64],
        yCoeff: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """
        This function calculates the length of each spline segment
        Parameters
        ----------
        trackCoeffs.xCoeff: np.ndarray, shape=(n, 4)
            x coefficients of the splines
        trackCoeffs.yCoeff: np.ndarray, shape=(n, 4)
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
        self,
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
        trackCoeffs.xCoeff: np.ndarray, shape=(n, 4)
            x coefficients of the splines
        trackCoeffs.yCoeff: np.ndarray, shape=(n, 4)
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
        self,
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
        trackUpBound = self.smoothTrack.path[
            :, :2
        ] + self.smoothTrack.trackCoeffs.normVectors * np.expand_dims(
            self.smoothTrack.path[:, 2], 1
        )
        trackUpBound = np.vstack((trackUpBound, trackUpBound[0]))

        trackLowBound = self.smoothTrack.path[
            :, :2
        ] - self.smoothTrack.trackCoeffs.normVectors * np.expand_dims(
            self.smoothTrack.path[:, 3], 1
        )
        trackLowBound = np.vstack((trackLowBound, trackLowBound[0]))
        return (trackUpBound, trackLowBound)

    def numpyToPath(self, pathArray: npt.NDArray[np.float64]) -> Path:
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
        path = Path()
        print(pathArray.shape[0])
        for point in pathArray:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.header.frame_id = path.header.frame_id = "map"
            pose.header.stamp = path.header.stamp = rospy.Time.now()
            path.poses.append(pose)
        return path

    def calcSplines(
        self, path: npt.NDArray[np.float64]
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64],]:
        """
        This function calculates the spline coefficients for a given path
        Parameters
        ----------
        path: np.ndarray, shape=(n, 2)
            Path as XY coordinates
        Returns
        -------
        xCoeffs: np.ndarray, shape=(n, 4)
            x coefficients of the splines
        yCoeffs: np.ndarray, shape=(n, 4)
            y coefficients of the splines
        """

        # get coefficients of spline
        tSpline = np.arange(0, path.shape[0])
        xPoints = np.array(path[:, 0])
        yPoints = np.array(path[:, 1])

        xSpline = interpolate.CubicSpline(tSpline, xPoints)
        ySpline = interpolate.CubicSpline(tSpline, yPoints)

        xCoeffs = np.rot90(xSpline.c, 3)
        yCoeffs = np.rot90(ySpline.c, 3)

        return xCoeffs, yCoeffs
