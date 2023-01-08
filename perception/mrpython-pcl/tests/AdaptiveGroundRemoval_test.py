"""
Tests for the AdaptiveGroundRemoval class
"""
# pylint: disable=all
import time
from typing import Generator, Any, no_type_check

import pcl
import pytest
import numpy as np

# import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.GroundRemoval.AdaptiveGroundRemoval import AdaptiveGroundRemoval


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    AdaptiveGroundRemoval.clear()  # setup
    yield
    AdaptiveGroundRemoval.clear()  # teardown


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    groundRemoval = AdaptiveGroundRemoval(50, 50, 0.5)
    groundRemoval2 = AdaptiveGroundRemoval(100, 100, 0.8)
    assert groundRemoval == groundRemoval2
    assert groundRemoval2.nGridCells[0] == 50


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the ConeClassifier class
    """
    with pytest.raises(TypeError):
        AdaptiveGroundRemoval(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly:
    nGridCellsX, nGridCellsY: positive integers
    distFromPlaneTh: positive float
    """
    runTestParams("40", 40, 0.2)
    runTestParams(40, "40", 0.2)
    runTestParams(40, 40, "0.2")
    runTestParams(-40, 40, 0.2)
    runTestParams(40, -40, 0.2)
    runTestParams(40, 40, -0.2)
    runTestParams(40.5, 40, 0.2)
    runTestParams(40, 40.5, 0.2)
    runTestParams(40, 40, 0)
    runTestParams(0, 40, 0.2)
    runTestParams(40, 0, 0.2)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testGroundEasy() -> None:
    """
    Clear ground plane with no noise
    """
    # Test parameters
    nTests = 10
    nPoints = 2000
    maxTime = 110  # ms
    maxErrors = 100

    tic = time.time()
    for _ in range(nTests):
        xySpace = np.random.uniform(-50, 50, (nPoints, 2))
        normal = np.random.uniform(-0.1, 0.1, (1, 2))
        z = np.sum(xySpace * normal, axis=1)
        points = np.hstack((xySpace, z.reshape(-1, 1)))

        randomPoints = np.random.normal(0, 3, (nPoints, 3))
        randomPoints[:, 2] = np.abs(randomPoints[:, 2])
        points = np.vstack((points, randomPoints))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        groundRemover = AdaptiveGroundRemoval(40, 40, 0.1)
        filtered = groundRemover.removeGround(cloud).to_array()
        assert (
            nPoints - maxErrors < filtered.shape[0] < nPoints + maxErrors
        )  # Removes most ground points, leaving some noise
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
    maxTime = 110  # ms
    maxErrors = 250

    tic = time.time()
    for _ in range(nTests):
        xySpace = np.random.uniform(-50, 50, (nPoints, 2))
        normal = np.random.uniform(-1, 1, (1, 2))
        z = np.sum(xySpace * normal, axis=1)
        points = np.hstack((xySpace, z.reshape(-1, 1)))
        points += np.random.normal(0, 0.07, points.shape)

        randomPoints = np.random.normal(0, 3, (nPoints, 3))
        points = np.vstack((points, randomPoints))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        groundRemover = AdaptiveGroundRemoval(40, 40, 0.4)
        filtered = groundRemover.removeGround(cloud).to_array()
        assert (
            nPoints - maxErrors < filtered.shape[0] < nPoints + maxErrors
        )  # Removes most ground points, leaving some noise
    toc = time.time()
    totalTime = (toc - tic) * 1000 / nTests

    assert totalTime < maxTime
