"""
Class to generate a track for the planning simulator
"""
from typing import Tuple
from dataclasses import dataclass
from math import pi, cos, sin, ceil

import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from scipy.optimize import minimize, Bounds


@dataclass
class TrackConfig:
    """
    Configuration for the track generator

    Parameters
    ----------
    length: float
        Desired length of track
    rmin: float
        Minimum corner radius
    rmax: float
        Maximum corner radius
    lmax: float
        Maximum straight length
    lmin: float
        Minimum straight length
    dthmax: float
        Maximum angle change of corner
    dthmin: float
        Minimum angle change of corner
    left: bool
        Orientation of track (Left-turning if True)
    width: float
        Track width
    """

    length: float = 200.0
    rmin: float = 5
    rmax: float = 15.0
    lmax: float = 20.0
    lmin: float = 2.0
    dthmin: float = pi / 6
    dthmax: float = pi
    left: bool = True
    width: float = 4.0


class Track:
    """
    Track object holds all parameters defining the track, as well as the
    constraints under which this track was designed.

    Parameters
    ----------
    crns: np.array
        Track lay-out
    lpar: np.array
        Optimized length parameters
    delTh: np.array
        Optimized angle changes
    """

    def __init__(
        self, trackConfig: TrackConfig, crns: npt.NDArray[np.bool_] = np.zeros((0,), dtype=bool)
    ):
        self.trackConfig = trackConfig
        self.crns = crns
        self.lpar = np.zeros((0,))
        self.delTh = np.zeros((0,))

        # boundaries
        self.xb1 = np.zeros((0,))
        self.xb2 = np.zeros((0,))
        self.yb1 = np.zeros((0,))
        self.yb2 = np.zeros((0,))

        # midline
        self.xMidline = np.zeros((0,))
        self.yMidline = np.zeros((0,))
        self.sMidline = np.zeros((0,))
        self.thetaMidline = np.zeros((0,))

        # cones
        self.xc1 = np.zeros((0,))
        self.xc2 = np.zeros((0,))

        self.optimized = False

    def solve(
        self, lparInit: npt.NDArray[np.float64], delThInit: npt.NDArray[np.float64], case: int = 0
    ) -> None:
        """
        Solves the optimization problem that ensures the track has the correct
        length, curvature, etc. using an SLSQP algorithm.
        - Case 0: maximizes curvature
        - Case 1: minimizes curvature
        - Case 2: only satisfies constraints

        Parameters
        ----------
        lparInit: np.array
            Initial guess for length parameters
        delThInit: np.array
            Initial guess for angle changes
        case: int
            Optimization case
        """

        nseg = len(lparInit)
        assert nseg == len(delThInit)
        assert nseg == len(self.crns)

        if case > 2:
            raise ValueError("Case number higher than 2")

        # Decide on objective function
        if case == 0:
            fobj = objMaxCurv
        elif case == 1:
            fobj = objMinCurv
        elif case == 2:
            fobj = objNone

        xInit = np.hstack((lparInit, delThInit))

        # equality constraints
        constr = {
            "type": "eq",
            "fun": eqConstr,
            "args": (self.crns, self.trackConfig.length, self.trackConfig.left),
        }

        # bounds
        lowerBounds = np.zeros(xInit.shape)
        upperBounds = np.zeros(xInit.shape)
        for i in range(0, nseg):
            if self.crns[i]:
                lowerBounds[i + nseg] = self.trackConfig.dthmin
                upperBounds[i + nseg] = self.trackConfig.dthmax
                if lparInit[i] > 0.0:
                    lowerBounds[i] = self.trackConfig.rmin
                    upperBounds[i] = self.trackConfig.rmax
                else:
                    upperBounds[i] = -self.trackConfig.rmin
                    lowerBounds[i] = -self.trackConfig.rmax
            else:
                lowerBounds[i] = self.trackConfig.lmin
                lowerBounds[i + nseg] = 0.0
                upperBounds[i] = self.trackConfig.lmax
                upperBounds[i + nseg] = 0.0

        bnds = Bounds(lowerBounds, upperBounds)

        soldict = minimize(
            fobj,
            xInit,
            args=(self.trackConfig.lmax, self.trackConfig.dthmax),
            method="SLSQP",
            jac=True,
            bounds=bnds,
            constraints=constr,
            tol=None,
            options=None,
        )

        self.lpar = soldict.x[0:nseg]
        self.delTh = soldict.x[nseg:]

        self.optimized = True

    def endpoint(self) -> Tuple[float, float, float]:
        """
        Returns endpoint of the track. If optimization is successful, should be the origin

        Returns
        -------
        float
            x-coordinate of endpoint
        float
            y-coordinate of endpoint
        float
            angle of endpoint
        """
        return compEndpoint(self.crns, self.lpar, self.delTh)

    def plot(
        self,
        cones: bool = False,
        aveDist: float = 3.0,
        show: bool = False,
        filename: str = "track.png",
    ) -> None:
        """
        Plots the track defined in the Track object.

        Parameters
        ----------
        cones: bool
            Whether to plot cones or not
        aveDist: float
            Average distance between cones
        show: bool
            Whether to show the plot or not
        filename: str
            Filename to save the plot to
        """

        if np.shape(self.xb1)[0] == 0:
            self.compTrackXY()

        if (np.shape(self.xc1)[0] == 0) and cones:
            self.populateCones(aveDist)

        # actually plot
        plt.figure()
        if self.trackConfig.left:
            plt.fill(self.xb1, self.yb1, "0.75")
            plt.fill(self.xb2, self.yb2, "w")
        else:
            plt.fill(self.xb2, self.yb2, "0.75")
            plt.fill(self.xb1, self.yb1, "w")

        plt.plot(self.xMidline, self.yMidline, "k--", linewidth=1)
        plt.plot(self.xb1, self.yb1, linewidth=2, color="k")
        plt.plot(self.xb2, self.yb2, linewidth=2, color="k")

        if cones:
            plt.plot(self.xc1, self.yc1, "ro")
            plt.plot(self.xc2, self.yc2, "go")

        plt.axis("equal")
        if show:
            plt.show()
        if filename is not None:
            plt.savefig(filename)

    def compTrackXY(self) -> None:
        """
        Computes track in x,y-space.
        """

        nplot = 50  # number of points used for corners

        nseg = len(self.crns)
        ncrns = sum(self.crns)
        npts = ncrns * nplot + (nseg - ncrns) * 2 + 1

        xmid = np.zeros((npts,))
        ymid = np.zeros((npts,))
        smid = np.zeros((npts,))

        theta = np.zeros((npts,))

        thcum = 0.0

        ind = 0

        for i in range(0, nseg):
            if self.crns[i]:
                phi = np.linspace(0.0, self.delTh[i], nplot)

                delx = abs(self.lpar[i]) * np.sin(phi)  # local coordinate frame
                dely = self.lpar[i] - self.lpar[i] * np.cos(phi)  # local coordinate frame

                # map to global coordinate frame
                xmid[(ind + 1) : (ind + nplot + 1)] = (
                    xmid[ind] + delx * cos(thcum) - dely * sin(thcum)
                )
                ymid[(ind + 1) : (ind + nplot + 1)] = (
                    ymid[ind] + dely * cos(thcum) + delx * sin(thcum)
                )

                # update cumulative angle
                thcum += np.sign(self.lpar[i]) * self.delTh[i]
                theta[(ind + 1) : (ind + nplot + 1)] = theta[ind] + np.sign(self.lpar[i]) * phi

                # update distance
                smid[(ind + 1) : (ind + nplot + 1)] = smid[ind] + abs(self.lpar[i]) * phi

                ind += nplot

            else:
                xmid[ind + 1] = xmid[ind]
                ymid[ind + 1] = ymid[ind]
                smid[ind + 1] = smid[ind]

                xmid[ind + 2] = xmid[ind] + self.lpar[i] * cos(thcum)
                ymid[ind + 2] = ymid[ind] + self.lpar[i] * sin(thcum)
                smid[ind + 2] = smid[ind] + self.lpar[i]

                theta[ind + 1] = theta[ind]
                theta[ind + 2] = theta[ind]

                ind += 2

        self.xb1 = xmid + self.trackConfig.width / 2 * np.sin(theta)
        self.yb1 = ymid - self.trackConfig.width / 2 * np.cos(theta)
        self.xb2 = xmid - self.trackConfig.width / 2 * np.sin(theta)
        self.yb2 = ymid + self.trackConfig.width / 2 * np.cos(theta)

        self.xMidline = xmid
        self.yMidline = ymid
        self.thetaMidline = theta
        self.sMidline = smid

    def populateCones(self, aveDist: float) -> None:
        """
        Populates track with cones.

        Parameters
        ----------
        aveDist: float
            Average distance between cones and the midline
        """
        nseg = len(self.crns)

        xc1 = np.zeros((0,))
        yc1 = np.zeros((0,))

        xc2 = np.zeros((0,))
        yc2 = np.zeros((0,))

        thcum = 0.0

        for i in range(0, nseg):
            if self.crns[i]:
                r1 = self.lpar[i] - self.trackConfig.width / 2
                r2 = self.lpar[i] + self.trackConfig.width / 2

                n1 = (
                    int(ceil(self.delTh[i] * abs(r1) / aveDist)) + 1
                )  # number of points used on left boundary
                n2 = (
                    int(ceil(self.delTh[i] * abs(r2) / aveDist)) + 1
                )  # number of points used on right boundary

                phi1 = np.linspace(0.0, self.delTh[i], n1)
                phi2 = np.linspace(0.0, self.delTh[i], n2)

                # delete first point
                phi1 = np.delete(phi1, 0)
                phi2 = np.delete(phi2, 0)

                delx1 = abs(r1) * np.sin(phi1)  # local coordinate frame
                dely1 = r1 - r1 * np.cos(phi1)  # local coordinate frame

                delx2 = abs(r2) * np.sin(phi2)  # local coordinate frame
                dely2 = r2 - r2 * np.cos(phi2)  # local coordinate frame

                # map to global coordinate frame
                x1 = delx1 * cos(thcum) - dely1 * sin(thcum)
                y1 = dely1 * cos(thcum) + delx1 * sin(thcum)

                x2 = delx2 * cos(thcum) - dely2 * sin(thcum)
                y2 = dely2 * cos(thcum) + delx2 * sin(thcum)

                if len(xc1) > 0:
                    x1 += xc1[-1]
                    y1 += yc1[-1]
                    x2 += xc2[-1]
                    y2 += yc2[-1]

                # update cumulative angle
                thcum += np.sign(self.lpar[i]) * self.delTh[i]

                # append
                xc1 = np.hstack([xc1, x1])
                yc1 = np.hstack([yc1, y1])
                xc2 = np.hstack([xc2, x2])
                yc2 = np.hstack([yc2, y2])

            else:
                n = int(ceil(self.lpar[i] / aveDist)) + 1

                xloc = np.linspace(0, self.lpar[i], n)
                xloc = np.delete(xloc, 0)

                x1 = xloc * cos(thcum)
                y1 = xloc * sin(thcum)
                x2 = xloc * cos(thcum)
                y2 = xloc * sin(thcum)

                if len(xc1) > 0:
                    x1 += xc1[-1]
                    y1 += yc1[-1]
                    x2 += xc2[-1]
                    y2 += yc2[-1]
                else:
                    y1 += self.trackConfig.width / 2
                    y2 -= self.trackConfig.width / 2

                # append
                xc1 = np.hstack([xc1, x1])
                yc1 = np.hstack([yc1, y1])
                xc2 = np.hstack([xc2, x2])
                yc2 = np.hstack([yc2, y2])

        self.xc1 = xc1
        self.xc2 = xc2
        self.yc1 = yc1
        self.yc2 = yc2


def eqConstr(
    x: npt.NDArray[np.float64], crns: npt.NDArray[np.bool_], leng: float, left: bool
) -> npt.NDArray[np.float64]:
    """
    Computes the value of the equality constraints for `x`.
    """
    constr = np.zeros((4,))

    nseg = int(len(x) / 2)

    # length constraint
    constr[0] = compLength(crns, x[0:nseg], x[nseg:])
    constr[0] -= leng

    # end point constraints and angle constraint
    constr[1], constr[2], constr[3] = compEndpoint(crns, x[0:nseg], x[nseg:])
    constr[3] -= (-1 + left * 2) * 2 * pi
    return constr


def compLength(
    crns: npt.NDArray[np.bool_], lpar: npt.NDArray[np.float64], delTh: npt.NDArray[np.float64]
) -> float:
    """
    Computes final length of track, defined by corner definition `crns`, length
    parameters `lpar`, and angle changes `delTh`.
    Also computes gradient of length with respect to design variables.
    """
    trlen = 0.0

    for corner, length, theta in zip(crns, lpar, delTh):
        if corner:
            trlen += abs(length) * theta
        else:
            trlen += length

    return trlen


def compEndpoint(
    crns: npt.NDArray[np.bool_], lpar: npt.NDArray[np.float64], delTh: npt.NDArray[np.float64]
) -> Tuple[float, float, float]:
    """
    Computes end point of track, defined by corner definition `crns`,
    length parameters `lpar`, and angle changes `delTh`.
    Also computes gradient with respect to design variables.

    Parameters
    ----------
    crns : ndarray
        Corner definition
    lpar : ndarray
        Length parameters
    delTh : ndarray
        Angle changes

    Returns
    -------
    float
        x-coordinate of end point
    float
        y-coordinate of end point
    float
        Cumulative angle
    """

    xend = 0.0
    yend = 0.0

    thcum = 0.0

    xend = 0.0
    yend = 0.0
    thcum = 0.0

    for idx, (corner, length, theta) in enumerate(zip(crns, lpar, delTh)):
        im1 = max(idx - 1, 0)
        if corner:
            delx = abs(length) * sin(theta)  # local coordinate frame
            dely = length - length * cos(theta)  # local coordinate frame

            # map to global coordinate frame
            xend += delx * cos(thcum) - dely * sin(thcum)
            yend += dely * cos(thcum) + delx * sin(thcum)

            # update cumulative angle
            thcum += np.sign(length) * theta
        else:
            xend += length * cos(thcum)
            yend += length * sin(thcum)

    return xend, yend, thcum


def compCurvature(
    lpar: npt.NDArray[np.float64], delTh: npt.NDArray[np.float64], lmax: float, dthmax: float
) -> Tuple[float, npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Computes track curvature and the gradient of track curvature with respect
    to the design variables.

    Parameters
    ----------
    lpars : ndarray
        Length parameters
    delTh : ndarray
        Angle changes
    lmax : float
        Maximum length parameter
    dthmax : float
        Maximum angle change
    Returns
    -------
    float
        Track curvature
    npt.NDArray[np.float64]
        Gradient of track curvature with respect to length parameters
    npt.NDArray[np.float64]
        Gradient of track curvature with respect to angle changes
    """

    curv = (np.linalg.norm(lpar).item() / lmax) ** 2 + (np.linalg.norm(delTh).item() / dthmax) ** 2

    dcurvdlpar = 2 * lpar / lmax**2
    dcurvddelth = 2 * delTh / dthmax**2

    return curv, dcurvdlpar, dcurvddelth


def objMaxCurv(
    x: npt.NDArray[np.float64], lmax: float, dthmax: float
) -> Tuple[float, npt.NDArray[np.float64]]:
    """
    Objective function for maximum curvature.

    Parameters
    ----------
    x: ndarray
        Design variables
    lmax : float
        Maximum length parameter
    dthmax : float
        Maximum angle change

    Returns
    -------
    float
        Negative * Track curvature
    np.ndarray
        Gradient of track curvature with respect to design variables

    """
    nseg = int(len(x) / 2)

    curv, dcdlpar, dcddelth = compCurvature(x[0:nseg], x[nseg:], lmax, dthmax)

    return -curv, np.hstack((-dcdlpar, -dcddelth))


def objMinCurv(
    x: npt.NDArray[np.float64], lmax: float, dthmax: float
) -> Tuple[float, npt.NDArray[np.float64]]:
    """
    Objective function for minimum curvature.

    Parameters
    ----------
    x: ndarray
        Design variables
    lmax : float
        Maximum length parameter
    dthmax : float
        Maximum angle change

    Returns
    -------
    float
        Track curvature
    np.ndarray
        Gradient of track curvature with respect to design variables

    """
    nseg = int(len(x) / 2)

    curv, dcdlpar, dcddelth = compCurvature(x[0:nseg], x[nseg:], lmax, dthmax)

    return curv, np.hstack((dcdlpar, dcddelth))


def objNone(
    x: npt.NDArray[np.float64], lmax: float, dthmax: float
) -> Tuple[float, npt.NDArray[np.float64]]:
    """
    Constant objective function.

    Parameters
    ----------
    x: ndarray
        Design variables
    lmax : float
        Maximum length parameter
    dthmax : float
        Maximum angle change

    Returns
    -------
    float
        1.0
    np.ndarray
        Gradient of 1.0 with respect to design variables (zeros)
    """
    return 1.0, np.zeros((len(x),))
