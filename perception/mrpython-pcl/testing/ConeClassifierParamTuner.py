"""
This module can be used to run tests on the ConeClassiier hyperparameters
"""
from typing import Tuple
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.ConeClassifier import ConeClassifier  # type: ignore


def plotPoints(points: npt.NDArray[np.float64]) -> None:
    """
    Plot 3D points using matplotlib
    """
    axis = plt.axes(projection="3d")
    axis.scatter3D(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap="Greens")
    plt.show()


def generateCone(
    nPoints: int, radius: float, height: float, coneCenter: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """
    Generate points falling on a cone with the given dimensions and center

    Parameters
    ----------
    nPoints: int
        Number of points to generate
    radius: float
    height: float
        Height of the vertex at the top of the cone
    coneCenter: np.array, shape=(3,)
        x,y,z location of the vertex at the top of the cone

    Returns
    -------
    points: np.array, shape=(nPoints, 3)
        Generated points
    """
    conePoints = np.random.uniform(-radius, radius, (nPoints, 2))
    x, y = conePoints.T
    z = -np.sqrt((x**2 + y**2) * height**2 / radius**2)
    points = np.hstack((x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)))
    points += coneCenter
    return points


def computeAvgLoss(
    radius: float,
    height: float,
    nPoints: int,
    variance: float,
    nTests: int = 100,
    isPlotPoints: bool = False,
) -> Tuple[float, float]:
    """
    Compute average losses (l2 and linearization) over multiple cones with different noises

    Parameters
    ----------
    radius: float
    height: float
    nPoints: int
        Number of points per cone
    variance: float
        variance of noise added
    nTests: int
        Number of cones to generate and average losses over
    isPlotPoints: bool
        Whether to plot the points generated for each test

    Returns
    -------
    avg_lin_loss: float
    avg_l2_loss: float
    """
    linLosses = []
    l2Losses = []
    coneClassifier = ConeClassifier(radius, height, 1, 10, 10)

    for _ in range(nTests):
        coneCenter = np.random.uniform(-15, 15, (3,))
        points = generateCone(nPoints, radius, height, coneCenter)
        points += np.random.normal(0, variance, points.shape)  # Added noise

        res, _ = coneClassifier.isCone(points, True)
        if isPlotPoints:
            plotPoints(points)
        linLosses.append(res[1])
        l2Losses.append(res[2])

    meanLinLosses: float = float(np.mean(linLosses))
    meanL2Losses: float = float(np.mean(l2Losses))

    return meanLinLosses, meanL2Losses


if __name__ == "__main__":
    RADIUS = 0.15
    HEIGHT = 0.4
    MAX_VARIANCE = 0.1
    PLOT_POINTS = False
    variancesToTest = np.arange(0.01, MAX_VARIANCE, 0.0005).tolist()
    nPointsList = [200]
    for numPoints in nPointsList:
        allLinLosses = []
        allL2Losses = []
        for test_variance in variancesToTest:
            losses = computeAvgLoss(RADIUS, HEIGHT, numPoints, test_variance, PLOT_POINTS)
            allLinLosses.append(losses[0])
            allL2Losses.append(losses[1])

        plt.title("Losses for " + str(numPoints) + " points")
        plt.plot(variancesToTest, allLinLosses, label="Lin Losses")
        plt.plot(variancesToTest, allL2Losses, label="L2 Losses")
        plt.legend()
        plt.show()
