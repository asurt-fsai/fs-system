"""
Tests for the ConeClassifier class
"""
# pylint: disable=all
import time
from typing import Generator, Any, no_type_check

import pytest

import numpy as np
import numpy.typing as npt

# import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.ConeClassifier import ConeClassifier


RADIUS = 0.15
HEIGHT = 0.4
L2LOSSTHRESH = 0.02
LINLOSSPERCENT = 1e-4
MINPOINTS = 10


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


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    ConeClassifier.clear()  # setup
    yield
    ConeClassifier.clear()  # teardown


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    coneClassifier = ConeClassifier(5, 5, 5, 5, 5)
    coneClassifier2 = ConeClassifier(5, 5, 10, 5, 5)
    assert coneClassifier == coneClassifier2
    assert coneClassifier2.minPoints == 5


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the ConeClassifier class
    """
    with pytest.raises(TypeError):
        ConeClassifier(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly (all positive, minPoints is an integer)
    """
    runTestParams(-5, 5, 5, 5, 5)
    runTestParams(5, -5, 5, 5, 5)
    runTestParams(5, 5, -5, 5, 5)
    runTestParams(5, 5, 5, -5, 5)
    runTestParams(5, 5, 5, 5, -5)
    runTestParams(5, 5, 5.5, 5, 5)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testConeEasy() -> None:
    """
    Cones generated with no noise
    """
    # test parameters
    nTests = 10
    numPoints = 4  # Hardest case (with 3 points, it fails mathematically)

    coneClassifier = ConeClassifier(
        RADIUS, HEIGHT, 4, L2LOSSTHRESH, LINLOSSPERCENT  # Use min points=4 for this test
    )

    totalTime = []
    dists = []
    for _ in range(nTests):  # Run test multiple times
        coneCenter = np.random.uniform(-15, 15, (3,))
        points = generateCone(numPoints, RADIUS, HEIGHT, coneCenter)

        # Run test
        tic = time.time()
        res, center = coneClassifier.isCone(points, True)
        toc = time.time()
        totalTime.append((toc - tic) * 1000)

        # Assertions
        if not res[0]:
            print(res)
        assert res[0]

        diff = np.linalg.norm(center - coneCenter)
        dists.append(diff)
        assert diff < 0.01

    print("Easy test report:")
    print("Avg dist to cone center: ", np.mean(dists))
    print("Avg time (ms):", np.mean(totalTime))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testConeNoisy() -> None:
    """
    Cones generated with added noise
    """
    # test parameters
    nTests = 50

    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)

    totalTime = []
    dists = []
    for _ in range(nTests):  # Run test multiple times
        coneCenter = np.random.uniform(-15, 15, (3,))
        numPoints = np.random.randint(11, 50)
        points = generateCone(numPoints, RADIUS, HEIGHT, coneCenter)
        points += np.random.normal(0, 0.02, points.shape)  # Added noise

        # Run test
        tic = time.time()
        res, center = coneClassifier.isCone(points, True)
        toc = time.time()
        totalTime.append((toc - tic) * 1000)

        # Plotting for debugging
        # plt.scatter(points[:,0], points[:,2])
        # plt.show()
        # plt.scatter(points[:,1], points[:,2])
        # plt.show()
        # print(res)

        # Assertions
        assert res[0]

        diff = np.linalg.norm(center - coneCenter)
        dists.append(diff)
        assert diff < 0.3

    print("Hard test report:")
    print("Avg dist to cone center: ", np.mean(dists))
    print("Avg time (ms):", np.mean(totalTime))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testTooNoisyCone() -> None:
    """
    Test cones with a lot of noise (shouldn't be detected as cones)
    """
    # test parameters
    nTests = 50

    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)

    totalTime = []
    for _ in range(nTests):  # Run test multiple times
        coneCenter = np.random.uniform(-15, 15, (3,))
        numPoints = np.random.randint(10, 50)
        points = generateCone(numPoints, RADIUS, HEIGHT, coneCenter)
        points += np.random.normal(0, 0.08, points.shape)  # Added noise

        # Run test
        tic = time.time()
        res, _ = coneClassifier.isCone(points)
        toc = time.time()
        totalTime.append((toc - tic) * 1000)

        # Assertions
        assert res == [False]

    print("Too noisy cone test report:")
    print("Avg time (ms):", np.mean(totalTime))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testNoCone() -> None:
    """
    Test random points (shouldn't be detected as cones)
    """
    # test parameters
    nTests = 50

    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)

    totalTime = []
    for _ in range(nTests):  # Run test multiple times
        numPoints = np.random.randint(10, 50)
        points = np.random.normal(0, 3, (numPoints, 3))  # Added noise

        # Run test
        tic = time.time()
        res, _ = coneClassifier.isCone(points)
        toc = time.time()
        totalTime.append((toc - tic) * 1000)

        # Assertions
        assert res == [False]

    print("No noisy cone test report:")
    print("Avg time (ms):", np.mean(totalTime))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testMinPointsFilter() -> None:
    """
    Test the minimum points filter
    """

    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)

    for nPoints in range(4, MINPOINTS):
        coneCenter = np.random.uniform(-15, 15, (3,))
        points = generateCone(nPoints, RADIUS, HEIGHT, coneCenter)
        res, center = coneClassifier.isCone(points)
        assert res == [False]
        assert center is None
