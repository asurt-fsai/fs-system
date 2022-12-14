"""
Tests for the MeanClusterer class
"""
import os
import sys
import time
import pytest
from typing import Generator, Any, Tuple, no_type_check

import numpy as np
import numpy.typing as npt

import matplotlib.pyplot as plt

# Import MeanClusterer
path, _ = os.path.split(os.path.split(__file__)[0])
path = os.path.join(path, "src")
sys.path.insert(0, path)
from modules.Clusterer.MeanClusterer import MeanClusterer  # pylint: disable=all

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
    points: np.array, shape=(N, 2)
        points generated
    cluster_centers: np.array, shape=(M, 2)
        Centers for all the clusters generated.
        Note: M could be less than nClusters to ensure minDistBetweenClusters
    """
    cluster_centers = np.random.uniform(-15, 15, (nClusters, 2))
    filtered_centers = [cluster_centers[0]]
    for center in cluster_centers:
        dists = np.linalg.norm(center.reshape(1, -1) - np.array(filtered_centers), axis=1)
        if np.min(dists) > minDistBetweenClusters:
            filtered_centers.append(center)
    cluster_centers = np.array(filtered_centers)
    nClusters = cluster_centers.shape[0]
    points = []
    for center in cluster_centers:
        n_points = np.random.randint(minPoints, maxPoints)
        new_points = np.random.normal(0, variance, (n_points, 2)) + center
        points.extend(new_points.tolist())

    toReturnPoints: npt.NDArray[np.float64] = np.array(points).astype(np.float64)
    return toReturnPoints, cluster_centers


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
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
        - all positive
        - cluster_minPoints and cluster_maxPoints are integers
        - cluster_minPoints < cluster_maxPoints
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
    np.random.seed(3)
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
        points, cluster_centers = generateClusters(
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
        dist_list = []
        for center in centers:
            dists = np.linalg.norm(cluster_centers - center.reshape(1, -1), axis=1)
            dist_list.append(np.min(dists))

        # Assert small distance to clusters and not missing too many clusters
        assert np.abs(centers.shape[0] - cluster_centers.shape[0]) < 5
        assert np.mean(dist_list) < 0.2

        avgClusDist.append(np.mean(dist_list))
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
    np.random.seed(3)
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
        points, cluster_centers = generateClusters(
            nClusters, minPoints, maxPoints, variance, minDistBetweenClusters
        )

        random_points = np.random.uniform(-15, 15, (200, 2))
        points = np.vstack((points, random_points))

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
        dist_list = []
        for center in centers:
            dists = np.linalg.norm(cluster_centers - center.reshape(1, -1), axis=1)
            dist_list.append(np.min(dists))

        # Assert small distance to clusters and not missing too many clusters
        assert np.abs(centers.shape[0] - cluster_centers.shape[0]) < 10
        assert np.mean(dist_list) < 0.3

        clustersMissed.append(np.abs(centers.shape[0] - nClusters))
        avgClusDist.append(np.mean(dist_list))

    assert totalTime / nTests < MAXTIMEFORHARDTEST
    print("Hard test report:")
    print("Avg time: ", totalTime / nTests)
    print("Avg dist to cluster: ", np.mean(avgClusDist))
    print("Avg number of clusters missed: ", np.mean(clustersMissed))
