"""
Tests for the Filter class
"""
import os
import sys
import time
import pytest
from typing import Generator, Any, no_type_check

import pcl
import numpy as np
import numpy.typing as npt

# Import Filter
path, _ = os.path.split(os.path.split(__file__)[0])
path = os.path.join(path, "src")
sys.path.insert(0, path)
from modules.Filter.Filter import Filter  # pylint: disable=all
from modules.Filter.GroundRemoval import (
    RansacGroundRemoval,
    AdaptiveGroundRemoval,
)  # pylint: disable=all

REPORTTIME = False
NTESTCASESPERTEST = 10
NPOINTSPERTEST = 1000


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    Filter.clear()  # setup
    RansacGroundRemoval.clear()
    yield
    Filter.clear()  # teardown
    RansacGroundRemoval.clear()


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 0.2, bounds, bounds)
    filter2 = Filter(groundRemovalMethod, 5, bounds, bounds)
    assert filter == filter2
    assert filter2.resconstructParam == 0.2


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the ConeClassifier class
    """
    with pytest.raises(TypeError):
        Filter(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly:
    groundRemovalMethod is a subclass of GroundRemovalMethod
    reconstructParam is a float > 0
    bounds is a dict with keys "x", "y", and "z" and values are lists of length 2
        Elements in the lists are floats with [min, max]
    carDimensions is a dict that doesn't necessarily have "z" key
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    wrong_bounds = {"x": [-10, 10], "z": [-10, 10]}
    wrong_bounds2 = {"x": [-10, 10], "y": [10, -10], "z": [-10, 10]}
    runTestParams(5, 5, bounds, bounds)
    runTestParams(groundRemovalMethod, -5, bounds, bounds)
    runTestParams(groundRemovalMethod, 5, "hi", bounds)
    runTestParams(groundRemovalMethod, 5, bounds, "hi")
    runTestParams(groundRemovalMethod, 5, wrong_bounds, bounds)
    runTestParams(groundRemovalMethod, 5, bounds, wrong_bounds)
    runTestParams(groundRemovalMethod, 5, wrong_bounds2, bounds)
    runTestParams(groundRemovalMethod, 5, bounds, wrong_bounds2)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testFilterViewableArea() -> None:
    """
    Test filter viewable area (removing points outside of the viewable area)
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)
    times = []
    for _ in range(NTESTCASESPERTEST):
        points = np.random.uniform(-15, 15, (NPOINTSPERTEST, 3)).astype(np.float32)
        cloud = pcl.PointCloud()
        cloud.from_array(points)

        start = time.time()
        filtered = filter.filterViewableArea(cloud)
        end = time.time()
        times.append((end - start) * 1000)

        assert np.all(filtered.to_array()[:, 0] >= -10)
        assert np.all(filtered.to_array()[:, 0] <= 10)
        assert np.all(filtered.to_array()[:, 1] >= -10)
        assert np.all(filtered.to_array()[:, 1] <= 10)
        assert np.all(filtered.to_array()[:, 2] >= -10)
        assert np.all(filtered.to_array()[:, 2] <= 10)

    if REPORTTIME:
        print("Test Filter Viewable Area: {} ms".format(np.mean(times)))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testRemoveCar() -> None:
    """
    Test remove car (removing points within a 2D box around the car)
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)
    times = []

    for _ in range(NTESTCASESPERTEST):
        points = np.random.uniform(-15, 15, (NPOINTSPERTEST, 3)).astype(np.float32)
        cloud = pcl.PointCloud()
        cloud.from_array(points)

        start = time.time()
        filtered = filter.removeCar(cloud)
        end = time.time()
        times.append((end - start) * 1000)

        filtered = filtered.to_array()
        inCarX = np.logical_and(filtered[:, 0] >= bounds["x"][0], filtered[:, 0] <= bounds["x"][1])
        inCarY = np.logical_and(filtered[:, 1] >= bounds["y"][0], filtered[:, 1] <= bounds["y"][1])
        inCar = np.logical_and(inCarX, inCarY)

        assert np.sum(inCar) == 0

    if REPORTTIME:
        print("Test Remove Car: {} ms".format(np.mean(times)))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testReconstruct() -> None:
    """
    Test reconstructing points around a cone's center
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    radius = 0.5
    filter = Filter(groundRemovalMethod, radius, bounds, bounds)
    pointCenters = np.random.uniform(-15, 15, (20, 2))
    pointCentersFiltered = [pointCenters[0]]
    for point_center in pointCenters:
        if np.min(
            np.linalg.norm(point_center.reshape(1, -1) - np.array(pointCentersFiltered), axis=1)
        ) > 2 * np.sqrt(2 * radius**2):
            pointCentersFiltered.append(point_center)
    pointCentersFiltered = np.array(pointCentersFiltered)
    pointCentersFiltered = np.hstack(
        (pointCentersFiltered, np.zeros((pointCentersFiltered.shape[0], 1)))
    )
    allPoints = []
    clusters = []
    for center in pointCentersFiltered:
        noise = np.random.normal(0, 0.2, (100, 2))
        noise = noise[np.linalg.norm(noise, axis=1) < radius]
        points = np.zeros((noise.shape[0], 3))
        points[:, :2] = noise
        points += center
        allPoints.extend(points.tolist())
        clusters.append(points.tolist())
    allPoints = np.array(allPoints).astype(np.float32)

    cloud = pcl.PointCloud()
    cloud.from_array(allPoints)

    for center, cluster in zip(pointCentersFiltered, clusters):
        cluster = np.array(cluster)
        filtered = filter.reconstruct(cloud, center[0], center[1])
        filtered = filtered.to_array()

        assert np.all(np.linalg.norm(filtered - center, axis=1) < np.sqrt(2 * radius**2))
        assert filtered.shape[0] == len(cluster)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testRansacGroundRemoval() -> None:
    """
    Easy ground removal test using the RansacGroundRemoval method
    """
    groundRemovalMethod = RansacGroundRemoval(0.0001)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)

    for _ in range(NTESTCASESPERTEST):
        x_y = np.random.uniform(-15, 15, (NPOINTSPERTEST, 2))
        n = np.random.uniform(-1, 1, (1, 2))
        z = np.sum(x_y * n, axis=1)
        points = np.hstack((x_y, z.reshape(-1, 1)))

        random_points = np.random.normal(0, 5, (NPOINTSPERTEST, 3))
        points = np.vstack((points, random_points))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        filter = RansacGroundRemoval(0.0001)
        filtered = filter.removeGround(cloud).to_array()
        assert filtered.shape[0] == NPOINTSPERTEST  # Removes all ground points, leaving only noise


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testAdaptiveGroundRemoval() -> None:
    """
    Easy ground removal test using the AdaptiveGroundRemoval method
    """
    groundRemovalMethod = AdaptiveGroundRemoval(40, 40, 0.2)
    maxErrors = 100
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)

    for _ in range(NTESTCASESPERTEST):
        x_y = np.random.uniform(-50, 50, (NPOINTSPERTEST, 2))
        n = np.random.uniform(-1, 1, (1, 2))
        z = np.sum(x_y * n, axis=1)
        points = np.hstack((x_y, z.reshape(-1, 1)))

        random_points = np.random.normal(0, 3, (NPOINTSPERTEST, 3))
        random_points[:, 2] = np.abs(random_points[:, 2])
        points = np.vstack((points, random_points))
        cloud = pcl.PointCloud()
        cloud.from_array(points.astype(np.float32))

        filtered = filter.removeGround(cloud).to_array()
        assert (
            NPOINTSPERTEST - maxErrors < filtered.shape[0] < NPOINTSPERTEST + maxErrors
        )  # Removes most ground points, leaving some noise


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testPassFilter() -> None:
    """
    Testing the pass filter (removing points beyond a certain value on an axis)
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)

    points = np.random.uniform(-15, 15, (500, 3)).astype(np.float32)
    cloud = pcl.PointCloud()
    cloud.from_array(points)

    # Test filter on x axis
    filteredPointsX = filter.passFilter(cloud, "x", 0, 5).to_array()

    # Test filter on y axis
    filteredPointsY = filter.passFilter(cloud, "y", 0, 5).to_array()

    # Test filter on z axis
    filteredPointsZ = filter.passFilter(cloud, "z", 0, 5).to_array()

    correctX = np.logical_and(filteredPointsX[:, 0] >= 0, filteredPointsX[:, 0] <= 5)
    correctY = np.logical_and(filteredPointsY[:, 1] >= 0, filteredPointsY[:, 1] <= 5)
    correctZ = np.logical_and(filteredPointsZ[:, 2] >= 0, filteredPointsZ[:, 2] <= 5)

    assert np.all(correctX)
    assert np.all(correctY)
    assert np.all(correctZ)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSubsample() -> None:
    """
    Test subsampling a point cloud (sampling only a ratio of points)
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)
    times = []

    for _ in range(NTESTCASESPERTEST):
        points = np.random.uniform(-15, 15, (NPOINTSPERTEST, 3)).astype(np.float32)
        cloud = pcl.PointCloud()
        cloud.from_array(points)

        start = time.time()
        subsampled = filter.subsample(cloud, 0.5)
        end = time.time()
        times.append((end - start) * 1000)

        assert subsampled.to_array().shape[0] == NPOINTSPERTEST // 2
    if REPORTTIME:
        print("Test Subsample: {} ms".format(np.mean(times)))


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testRemoveIntensity() -> None:
    """
    Test removing the intensity channel from a point cloud
    """
    groundRemovalMethod = RansacGroundRemoval(0.2)
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(groundRemovalMethod, 5, bounds, bounds)
    times = []

    for _ in range(NTESTCASESPERTEST):
        points = np.random.uniform(-15, 15, (NPOINTSPERTEST, 4)).astype(np.float32)
        cloud = pcl.PointCloud_PointXYZI()
        cloud.from_array(points)

        start = time.time()
        cloud = filter.removeIntensity(cloud)
        end = time.time()
        times.append((end - start) * 1000)

        assert cloud.to_array().shape[1] == 3
    if REPORTTIME:
        print("Test Remove Intensity: {} ms".format(np.mean(times)))
