"""
Tests for the RansacGroundRemoval class
"""
# pylint: disable=all
import time
from typing import Generator, Any, no_type_check

import pcl
import pytest
import numpy as np

# import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.GroundRemoval.RansacGroundRemoval import RansacGroundRemoval


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    RansacGroundRemoval.clear()  # setup
    yield
    RansacGroundRemoval.clear()  # teardown


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    groundRemoval = RansacGroundRemoval(0.5)
    groundRemoval2 = RansacGroundRemoval(0.2)
    assert groundRemoval == groundRemoval2
    assert groundRemoval2.ransacTh == 0.5


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the ConeClassifier class
    """
    with pytest.raises(TypeError):
        RansacGroundRemoval(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly (ransacTh: float > 0)
    """
    runTestParams(0)
    runTestParams(-5)
    runTestParams("hi")


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testGroundEasy() -> None:
    """
    Clear ground plane with no noise
    """
    # Test parameters
    nTests = 10
    nPoints = 2000
    maxTime = 10  # ms

    tic = time.time()
    for _ in range(nTests):
        xySpace = np.random.uniform(-15, 15, (nPoints, 2))
        normal = np.random.uniform(-1, 1, (1, 2))
        z = np.sum(xySpace * normal, axis=1)
        points = np.hstack((xySpace, z.reshape(-1, 1)))

        randomPoints = np.random.normal(0, 5, (nPoints, 3))
        points = np.vstack((points, randomPoints))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        groundRemover = RansacGroundRemoval(0.0001)
        filtered = groundRemover.removeGround(cloud).to_array()
        assert filtered.shape[0] == nPoints  # Removes all ground points, leaving only noise
    toc = time.time()
    totalTime = (toc - tic) * 1000 / nTests

    assert totalTime < maxTime


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testGroundHard() -> None:
    """
    Ground plane with noise
    """
    # Test parameters
    nTests = 10
    nPoints = 2000
    maxTime = 10  # ms
    maxErrors = 70

    tic = time.time()
    for _ in range(nTests):
        xySpace = np.random.uniform(-15, 15, (nPoints, 2))
        normal = np.random.uniform(-1, 1, (1, 2))
        z = np.sum(xySpace * normal, axis=1)
        points = np.hstack((xySpace, z.reshape(-1, 1)))
        points += np.random.normal(0, 0.07, points.shape)

        randomPoints = np.random.normal(0, 5, (nPoints, 3))
        points = np.vstack((points, randomPoints))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        groundRemover = RansacGroundRemoval(0.2)
        filtered = groundRemover.removeGround(cloud).to_array()
        assert (
            nPoints - maxErrors < filtered.shape[0] < nPoints + maxErrors
        )  # Removes most ground points, leaving some noise
    toc = time.time()
    totalTime = (toc - tic) * 1000 / nTests

    assert totalTime < maxTime
