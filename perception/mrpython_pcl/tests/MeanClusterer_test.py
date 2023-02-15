"""
Tests for the MeanClusterer class
"""
# pylint: disable=all
import time
from typing import Generator, Any, Tuple, no_type_check

import pytest
import numpy as np
import numpy.typing as npt

import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.Clusterer.MeanClusterer import MeanClusterer

ENABLEPLOTTING = False
MAXTIMEFOREASYTEST = 30
MAXTIMEFORHARDTEST = 50


def generateClusters(
    nClusters: int, minPoints: int, maxPoints: int, variance: float, minDistBetweenClusters: float
) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """
    Generate a random set of clusters using given parameters

    Parameters
    ----------
    nClusters: int
        Number of clusters to generate
    minPoints, maxPoints: int, int
        Minimum and Maximum number of points per cluster
    variance: float
        The variance of points within each cluster
    minDistBetweenClusters: float
        Clusters are ensured to have at least a distance of minDistBetweenClusters between them

    Returns
    -------
    points: np.array, shape=(N, 3)
        points generated, all z values are zero
    clusterCenters: np.array, shape=(M, 2)
        Centers for all the clusters generated.
        Note: M could be less than nClusters to ensure minDistBetweenClusters
    """
    clusterCenters = np.random.uniform(-15, 15, (nClusters, 2))
    filteredCenters = [clusterCenters[0]]
    for center in clusterCenters:
        dists = np.linalg.norm(center.reshape(1, -1) - np.array(filteredCenters), axis=1)
        if np.min(dists) > minDistBetweenClusters:
            filteredCenters.append(center)
    clusterCenters = np.array(filteredCenters)
    nClusters = clusterCenters.shape[0]
    points = []
    for center in clusterCenters:
        nPoints = np.random.randint(minPoints, maxPoints)
        newPoints = np.random.normal(0, variance, (nPoints, 2)) + center
        points.extend(newPoints.tolist())

    toReturnPoints: npt.NDArray[np.float64] = np.array(points).astype(np.float64)
    toReturnPoints = np.hstack((toReturnPoints, np.zeros((toReturnPoints.shape[0], 1))))
    return toReturnPoints, clusterCenters


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    MeanClusterer.clear()  # setup
    yield
    MeanClusterer.clear()  # teardown


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    clusterer = MeanClusterer([80, 80], 2, 8, 5)
    clusterer2 = MeanClusterer([80, 80], 4, 8, 5)
    assert clusterer == clusterer2
    assert clusterer2.nmsRadius == 2


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the ConeClassifier class
    """
    with pytest.raises(TypeError):
        MeanClusterer(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly:
        - GridCells is a list of two ellements, all positive integers
        - nmsRadius is a positive float
        - nIters is a positive integer
        - minClusterPoints is a positive integer

    """
    runTestParams("hi", 2, 8, 5)
    runTestParams([-100, 100], 2, 8, 5)
    runTestParams([100, -100], 2, 8, 5)
    runTestParams([100.5, 100], 2, 8, 5)
    runTestParams([100, 100.5], 2, 8, 5)
    runTestParams([100, 100], -2, 8, 5)
    runTestParams([100, 100], 2, 8.5, 5)
    runTestParams([100, 100], 2, -8, 5)
    runTestParams([100, 100], 2, 8, 5.5)
    runTestParams([100, 100], 2, 8, -5)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testClusteringEasy() -> None:
    """
    Easy case of clustering with low variance in each cluster and no noisy points
    """
    # Test Parameters
    nTests = 20
    minDistBetweenClusters = 2

    clusterer = MeanClusterer([40, 40], 0.4, 2, 3)
    totalTime = 0
    avgClusDist = []
    clustersMissed = []
    for _ in range(nTests):  # Number of tests
        nClusters = np.random.randint(50, 100)
        minPoints, maxPoints = 500, 1000
        variance = 0.05
        points, clusterCenters = generateClusters(
            nClusters, minPoints, maxPoints, variance, minDistBetweenClusters
        )

        tic = time.time()
        centers = clusterer.cluster(points)
        toc = time.time()
        totalTime += (toc - tic) * 1000

        # Plotting for debugging
        if ENABLEPLOTTING:
            plt.scatter(points[:, 0], points[:, 1])
            plt.scatter(centers[:, 0], centers[:, 1])
            plt.show()
            print(nClusters, centers.shape[0])

        # Compute distance to ground truth cluster centers
        distsList = []
        for center in centers:
            dists = np.linalg.norm(clusterCenters - center.reshape(1, -1), axis=1)
            distsList.append(np.min(dists))

        # Assert small distance to clusters and not missing too many clusters
        assert np.abs(centers.shape[0] - clusterCenters.shape[0]) < 5
        assert np.mean(distsList) < 0.2

        avgClusDist.append(np.mean(distsList))
        clustersMissed.append(np.abs(centers.shape[0] - nClusters))

    assert totalTime / nTests < MAXTIMEFOREASYTEST
    print("Easy test report:")
    print("Avg time: ", totalTime / nTests)
    print("Avg dist to cluster: ", np.mean(avgClusDist))
    print("Avg number of clusters missed: ", np.mean(clustersMissed))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testClusteringHard() -> None:
    """
    A hard case of clustering (higher variance with added noisy points)
    """
    # Test Parameters
    nTests = 20
    minDistBetweenClusters = 0.8

    clusterer = MeanClusterer([40, 40], 0.3, 2, 5)
    totalTime = 0
    avgClusDist = []
    clustersMissed = []
    for _ in range(nTests):  # Number of tests
        nClusters = np.random.randint(50, 100)
        minPoints, maxPoints = 50, 100
        variance = 0.2
        points, clusterCenters = generateClusters(
            nClusters, minPoints, maxPoints, variance, minDistBetweenClusters
        )

        randomPoints = np.random.uniform(-15, 15, (200, 3))
        points = np.vstack((points, randomPoints))

        tic = time.time()
        centers = clusterer.cluster(points)
        toc = time.time()
        totalTime += (toc - tic) * 1000

        # Plotting for debugging
        if ENABLEPLOTTING:
            plt.scatter(points[:, 0], points[:, 1])
            plt.scatter(centers[:, 0], centers[:, 1])
            plt.show()
            print(nClusters, centers.shape[0])

        # Compute distance to ground truth cluster centers
        distsList = []
        for center in centers:
            dists = np.linalg.norm(clusterCenters - center.reshape(1, -1), axis=1)
            distsList.append(np.min(dists))

        # Assert small distance to clusters and not missing too many clusters
        assert np.abs(centers.shape[0] - clusterCenters.shape[0]) < 10
        assert np.mean(distsList) < 0.3

        clustersMissed.append(np.abs(centers.shape[0] - nClusters))
        avgClusDist.append(np.mean(distsList))

    assert totalTime / nTests < MAXTIMEFORHARDTEST
    print("Hard test report:")
    print("Avg time: ", totalTime / nTests)
    print("Avg dist to cluster: ", np.mean(avgClusDist))
    print("Avg number of clusters missed: ", np.mean(clustersMissed))
