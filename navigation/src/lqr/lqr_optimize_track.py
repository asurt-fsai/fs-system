"""
This file contains the optimization function for the raceline generation.
"""
import quadprog
import numpy.typing as npt
import numpy as np
import scipy
from .track_classes import SolverMatrices, Track

VEHICLE_WIDTH: float = 2
CURVATURE_BOUNDARIES: float = 0.2


def setupMatrices(
    track: Track,
    mat: SolverMatrices,
) -> None:
    """
    This function sets up the matrices for the optimization problem.

    Parameters
    ----------
    track : Track
        The track data.
    mat : SolverMatrices
        The helper matrices for the optimization problem.
    """
    # create extraction matrix for b_i coefficients used in gradient
    extMatB = np.zeros((track.smooth.noPoints, track.smooth.noSplines * 4), dtype=int)

    for i in range(track.smooth.noSplines):
        extMatB[i, i * 4 + 1] = 1  # 1 * b_ix = E_x * x

    # create extraction matrix -> only c_i coefficients of the
    # solved linear equation system are needed for curvature information
    extMatC = np.zeros((track.smooth.noPoints, track.smooth.noSplines * 4), dtype=int)

    for i in range(track.smooth.noSplines):
        extMatC[i, i * 4 + 2] = 2  # 2 * c_ix = D_x * x

    # ax=b --> (track.smooth.alpha)*(T_C) = (extMatC)
    tempTC = scipy.sparse.linalg.spsolve(track.smooth.alpha.T, extMatC.T)
    mat.matT[0] = tempTC.T

    # set up matMX and matMY matrices
    matMX = np.zeros((track.smooth.noSplines * 4, track.smooth.noPoints))
    matMY = np.zeros((track.smooth.noSplines * 4, track.smooth.noPoints))

    for i in range(track.smooth.noSplines):
        j = i * 4

        if i < track.smooth.noPoints - 1:
            matMX[j, i] = track.smooth.normVectors[i, 0]
            matMX[j + 1, i + 1] = track.smooth.normVectors[i + 1, 0]

            matMY[j, i] = track.smooth.normVectors[i, 1]
            matMY[j + 1, i + 1] = track.smooth.normVectors[i + 1, 1]
        else:
            matMX[j, i] = track.smooth.normVectors[i, 0]
            matMX[j + 1, 0] = track.smooth.normVectors[0, 0]  # close spline

            matMY[j, i] = track.smooth.normVectors[i, 1]
            matMY[j + 1, 0] = track.smooth.normVectors[0, 1]

    # set up mat.matQ[0] and mat.matQ[1] matrices including the point coordinate information
    mat.matQ[0] = np.zeros((track.smooth.noSplines * 4, 1))
    mat.matQ[1] = np.zeros((track.smooth.noSplines * 4, 1))

    for i in range(track.smooth.noSplines):
        j = i * 4

        if i < track.smooth.noPoints - 1:
            mat.matQ[0][j, 0] = track.smooth.track[i, 0]
            mat.matQ[0][j + 1, 0] = track.smooth.track[i + 1, 0]

            mat.matQ[1][j, 0] = track.smooth.track[i, 1]
            mat.matQ[1][j + 1, 0] = track.smooth.track[i + 1, 1]
        else:
            mat.matQ[0][j, 0] = track.smooth.track[i, 0]
            mat.matQ[0][j + 1, 0] = track.smooth.track[0, 0]

            mat.matQ[1][j, 0] = track.smooth.track[i, 1]
            mat.matQ[1][j + 1, 0] = track.smooth.track[0, 1]

    # set up mat.matP[0], mat.matP[1], mat.matP[2] matrices
    tempTB = scipy.sparse.linalg.spsolve(track.smooth.alpha.T, extMatB.T)
    matTB = tempTB.T
    primes = np.array([0, 0, 0, 0, 0])
    primes[0] = np.eye(track.smooth.noPoints, track.smooth.noPoints) * np.matmul(matTB, mat.matQ[0])
    primes[1] = np.eye(track.smooth.noPoints, track.smooth.noPoints) * np.matmul(matTB, mat.matQ[1])

    primes[2] = np.power(primes[0], 2)
    primes[3] = np.power(primes[3], 2)
    primes[4] = -2 * np.matmul(primes[0], primes[3])
    curvDen = np.power(primes[2] + primes[3], 1.5)  # calculate curvature denominator
    curvPart = np.divide(
        1, curvDen, out=np.zeros_like(curvDen), where=curvDen != 0
    )  # divide where not zero (diag elements)
    curvPartSq = np.power(curvPart, 2)
    mat.matP[0] = np.matmul(curvPartSq, primes[3])
    mat.matP[2] = np.matmul(curvPartSq, primes[2])
    mat.matP[1] = np.matmul(curvPartSq, primes[4])

    # SET UP FINAL MATRICES FOR SOLVER
    mat.matT[1] = np.matmul(mat.matT[0], matMX)
    mat.matT[2] = np.matmul(mat.matT[0], matMY)


def optimizeMinCurve(
    track: Track,
    mat: SolverMatrices,
) -> npt.NDArray[np.float64]:
    """
    This function optimizes the minimum curvature of the track.
    It uses the scipy.optimize.minimize function to minimize the cost function.
    The cost function is the sum of the quadratic cost matrices.
    The function returns the optimized track coordinates.

    Parameters
    ----------
    track : Track
        Track object containing the track information.
    mat : SolverMatrices
        SolverMatrices object containing the helper matrices for the solver.

    Returns
    -------
    alphaMinCurve : npt.NDArray[np.float64]
        Array containing the optimized track alphas
        which are going to be multiplied by the normal vector
        to get the raceline coordinates
    """
    tempCostMat = np.array([0, 0, 0])
    tempCostMat[0] = np.matmul(mat.matT[1].T, np.matmul(mat.matP[0], mat.matT[1]))
    tempCostMat[1] = np.matmul(mat.matT[2].T, np.matmul(mat.matP[1], mat.matT[1]))
    tempCostMat[2] = np.matmul(mat.matT[2].T, np.matmul(mat.matP[2], mat.matT[2]))
    costMatQuad = tempCostMat[0] + tempCostMat[1] + tempCostMat[2]
    # make costMatQuad symmetric(because solver used needs symmetrical)

    costMatQuad = (costMatQuad + costMatQuad.T) / 2
    tempCostMat = np.array([0, 0, 0])
    tempCostMat[0] = 2 * np.matmul(
        np.matmul(mat.matQ[0].T, mat.matT[0].T), np.matmul(mat.matP[0], mat.matT[1])
    )
    tempCostMat[1] = np.matmul(
        np.matmul(mat.matQ[0].T, mat.matT[0].T), np.matmul(mat.matP[1], mat.matT[2])
    ) + np.matmul(np.matmul(mat.matQ[1].T, mat.matT[0].T), np.matmul(mat.matP[1], mat.matT[1]))
    tempCostMat[2] = 2 * np.matmul(
        np.matmul(mat.matQ[1].T, mat.matT[0].T), np.matmul(mat.matP[2], mat.matT[2])
    )
    costMat = tempCostMat[0] + tempCostMat[1] + tempCostMat[2]
    costMat = np.squeeze(costMat)  # remove non-singleton dimensions

    # CURVATURE(KAPPA) CONSTRAINTS
    matCurvCons = np.array([0, 0])
    matCurvCons[0] = np.matmul(mat.curvPart, mat.matPrime[1])
    matCurvCons[1] = np.matmul(mat.curvPart, mat.matPrime[0])

    # this part is multiplied by alpha within the optimization
    curvature = np.matmul(matCurvCons[1], mat.matT[2]) - np.matmul(matCurvCons[0], mat.matT[1])

    # original curvature part (static part)
    curvReference = np.matmul(matCurvCons[1], np.matmul(mat.matT[0], mat.matQ[1]))
    curvReference -= np.matmul(matCurvCons[0], np.matmul(mat.matT[0], mat.matQ[0]))

    upperCon = np.ones((track.smooth.noPoints, 1)) * CURVATURE_BOUNDARIES - curvReference
    lowerCon = -(np.ones((track.smooth.noPoints, 1)) * -CURVATURE_BOUNDARIES - curvReference)
    constrains = np.append(upperCon, lowerCon)
    # Solve a Quadratic Program defined as:
    #    minimize
    #        (1/2) * alpha.T * costMatQuad * alpha + costMat.T * alpha
    #    subject to
    #        constCoeff * alpha <= constrains

    # calculate allowed deviation from refline
    maxDevRight = track.smooth.track[:, 2] - (VEHICLE_WIDTH / 2)
    maxDevLeft = track.smooth.track[:, 3] - (VEHICLE_WIDTH / 2)

    # consider value boundaries (-maxDevLeft <= alpha <= maxDevRight)
    constCoeff = np.vstack(
        (np.eye(track.smooth.noPoints), -np.eye(track.smooth.noPoints), curvature, -curvature)
    )
    constrains = np.append(constrains, maxDevRight)
    constrains = np.append(constrains, maxDevLeft)
    # solve problem
    alphaMinCurve: npt.NDArray[np.float64] = quadprog.solve.QP(
        costMatQuad, -costMat, -constCoeff.T, -constrains, 0
    )[0]

    return alphaMinCurve


if __name__ == "__main__":

    pass
