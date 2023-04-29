"""
.
"""
import numpy as np
import numpy.typing as npt
import scipy
from lqr import SmoothTrack


class SolverMatrices:  # pylint: disable=too-few-public-methods
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

    def __init__(self, track: SmoothTrack) -> None:
        self.matP: npt.NDArray[np.float64] = np.array([None, None, None])
        self.matPrime: npt.NDArray[np.float64] = np.array([None, None])
        self.matT: npt.NDArray[np.float64] = np.array([None, None, None])
        self.matQ: npt.NDArray[np.float64] = np.array([None, None])
        self.curvPart: npt.NDArray[np.float64] = np.array(None)
        self.setupMatrices(track)

    def setupMatrices(self, track: SmoothTrack) -> None:
        """
        This function sets up the matrices for the optimization problem.

        Parameters
        ----------
        track : Track
            The track data.
        self : SolverMatrices
            The helper matrices for the optimization problem.
        """
        # create extraction matrix for b_i coefficients used in gradient
        extMatB = np.zeros((track.noPoints, track.noSplines * 4), dtype=int)
        for i in range(track.noSplines):
            extMatB[i, i * 4 + 1] = 1  # 1 * b_ix = E_x * x
        # create extraction matrix -> only c_i coefficients of the
        # solved linear equation system are needed for curvature information
        extMatC = np.zeros((track.noPoints, track.noSplines * 4), dtype=int)

        for i in range(track.noSplines):
            extMatC[i, i * 4 + 2] = 2  # 2 * c_ix = D_x * x
        # ax=b --> (track.trackCoeffs.alpha)*(T_C) = (extMatC)
        tempTC = scipy.sparse.linalg.spsolve(track.trackCoeffs.alpha.T, extMatC.T)
        self.matT[0] = tempTC.T
        # set up matMX and matMY matrices
        matMX = np.zeros((track.noSplines * 4, track.noPoints))
        matMY = np.zeros((track.noSplines * 4, track.noPoints))

        for i in range(track.noSplines):
            j = i * 4

            if i < track.noPoints - 1:
                matMX[j, i] = track.trackCoeffs.normVectors[i, 0]
                matMX[j + 1, i + 1] = track.trackCoeffs.normVectors[i + 1, 0]

                matMY[j, i] = track.trackCoeffs.normVectors[i, 1]
                matMY[j + 1, i + 1] = track.trackCoeffs.normVectors[i + 1, 1]
            else:
                matMX[j, i] = track.trackCoeffs.normVectors[i, 0]
                matMX[j + 1, 0] = track.trackCoeffs.normVectors[0, 0]  # close spline

                matMY[j, i] = track.trackCoeffs.normVectors[i, 1]
                matMY[j + 1, 0] = track.trackCoeffs.normVectors[0, 1]

        # set up self.matQ[0] and self.matQ[1] matrices including the point coordinate information
        self.matQ[0] = np.zeros((track.noSplines * 4, 1))
        self.matQ[1] = np.zeros((track.noSplines * 4, 1))

        for i in range(track.noSplines):
            j = i * 4

            if i < track.noPoints - 1:
                self.matQ[0][j, 0] = track.path[i, 0]
                self.matQ[0][j + 1, 0] = track.path[i + 1, 0]

                self.matQ[1][j, 0] = track.path[i, 1]
                self.matQ[1][j + 1, 0] = track.path[i + 1, 1]
            else:
                self.matQ[0][j, 0] = track.path[i, 0]
                self.matQ[0][j + 1, 0] = track.path[0, 0]

                self.matQ[1][j, 0] = track.path[i, 1]
                self.matQ[1][j + 1, 0] = track.path[0, 1]

        # set up self.matP[0], self.matP[1], self.matP[2] matrices
        tempTB = scipy.sparse.linalg.spsolve(track.trackCoeffs.alpha.T, extMatB.T)
        matTB = tempTB.T
        self.matPrime = np.array([None, None, None, None, None])
        self.matPrime[0] = np.eye(track.noPoints, track.noPoints) * np.matmul(matTB, self.matQ[0])
        self.matPrime[1] = np.eye(track.noPoints, track.noPoints) * np.matmul(matTB, self.matQ[1])

        self.matPrime[2] = np.power(self.matPrime[0], 2)
        self.matPrime[3] = np.power(self.matPrime[1], 2)
        self.matPrime[4] = -2 * np.matmul(self.matPrime[0], self.matPrime[1])
        curvDen = np.power(
            self.matPrime[2] + self.matPrime[3], 1.5
        )  # calculate curvature denominator
        self.curvPart = np.divide(
            1, curvDen, out=np.zeros_like(curvDen), where=curvDen != 0
        )  # divide where not zero (diag elements)
        curvPartSq = np.power(self.curvPart, 2)
        self.matP[0] = np.matmul(curvPartSq, self.matPrime[3])
        self.matP[2] = np.matmul(curvPartSq, self.matPrime[2])
        self.matP[1] = np.matmul(curvPartSq, self.matPrime[4])

        # SET UP FINAL MATRICES FOR SOLVER
        self.matT[1] = np.matmul(self.matT[0], matMX)
        self.matT[2] = np.matmul(self.matT[0], matMY)
