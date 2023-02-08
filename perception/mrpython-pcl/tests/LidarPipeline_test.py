"""
Tests for the LidarPipeline class
"""
# pylint: disable=all
import os
import time
import pytest
from typing import Generator, Any, no_type_check

import pcl
import numpy as np
import numpy.typing as npt

import matplotlib.pyplot as plt

from mrpython_pcl.LidarPipeline.LidarPipeline import LidarPipeline
from mrpython_pcl.LidarPipeline.Filter.Filter import Filter
from mrpython_pcl.LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from mrpython_pcl.LidarPipeline.ConeClassifier import ConeClassifier
from mrpython_pcl.LidarPipeline.Clusterer.MeanClusterer import MeanClusterer

RADIUS = 0.228
HEIGHT = 0.4
L2LOSSTHRESH = 0.02
LINLOSSPERCENT = 1e-4
MINPOINTS = 5
MEAN_CLUSTER_N_ITERS = 3
MEAN_CLUSTER_N_GRID_CELLS = [40, 40]
RANSAC_GROUND_TH = 0.2
REPORT = True


def createPipeline() -> LidarPipeline:
    """
    Creates a LidarPipeline object with good parameters for testing

    Returns
    -------
    LidarPipeline
        The created object
    """
    viewBounds = {"x": [-10, 10], "y": [-6, 6], "z": [-2, 2]}
    carBounds = {"x": [-2, 0], "y": [-0.75, 0.75]}
    filter = Filter(SimpleGroundRemoval([0, 0, -1, -0.05], 0.1), RADIUS, viewBounds, carBounds)

    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)

    clusterer = MeanClusterer(MEAN_CLUSTER_N_GRID_CELLS, RADIUS, MEAN_CLUSTER_N_ITERS, MINPOINTS)

    return LidarPipeline(filter, clusterer, coneClassifier)


@no_type_check
@pytest.fixture
def setUpTearDown() -> Generator[Any, Any, Any]:
    """
    Setup and teardown for all tests (mainly to clear the singleton)
    """
    np.random.seed(3)
    LidarPipeline.clear()  # setup
    Filter.clear()
    SimpleGroundRemoval.clear()
    ConeClassifier.clear()
    MeanClusterer.clear()
    yield
    LidarPipeline.clear()  # teardown
    Filter.clear()
    SimpleGroundRemoval.clear()
    ConeClassifier.clear()
    MeanClusterer.clear()


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testSingleton() -> None:
    """
    Ensure only the first instantiation of the class is used
    """
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(SimpleGroundRemoval([0, 0, -1, -0.1], 0.1), RADIUS, bounds, bounds)
    clusterer = MeanClusterer([40, 40], RADIUS, 2, MINPOINTS)
    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)
    lidarPipeline = LidarPipeline(filter, clusterer, coneClassifier, None, 0)
    lidarPipeline2 = LidarPipeline(filter, clusterer, coneClassifier, None, 0.2)
    assert lidarPipeline == lidarPipeline2
    assert lidarPipeline2.lidarHeight == 0


def runTestParams(*args, **kwargs) -> None:  # type: ignore
    """
    Tests input validation for the LidarPipeline class
    """
    with pytest.raises(TypeError):
        LidarPipeline(*args, **kwargs)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testParams() -> None:
    """
    Ensure parameter validation is done correctly:
        - classes are correct types
        - lidarHeight is a positive float (the lidar can't be underground)
    """
    bounds = {"x": [-10, 10], "y": [-10, 10], "z": [-10, 10]}
    filter = Filter(SimpleGroundRemoval([0, 0, -1, -0.1], 0.1), RADIUS, bounds, bounds)
    clusterer = MeanClusterer([40, 40], RADIUS, 2, MINPOINTS)
    coneClassifier = ConeClassifier(RADIUS, HEIGHT, MINPOINTS, L2LOSSTHRESH, LINLOSSPERCENT)
    runTestParams(coneClassifier, clusterer, coneClassifier)
    runTestParams(filter, coneClassifier, coneClassifier)
    runTestParams(filter, clusterer, clusterer)
    runTestParams(filter, clusterer, coneClassifier, None, "hi")
    runTestParams(filter, clusterer, coneClassifier, None, -1)


@no_type_check
@pytest.mark.usefixtures("setUpTearDown")
def testEasy() -> None:
    """
    Cones from a point cloud with no noise
    The point cloud wasn't preprocessed in any way
    """
    packagePath, _ = os.path.split(os.path.split(__file__)[0])
    noiseVar = 0.0001
    lidarPipeline = createPipeline()

    saved_pc = np.load(os.path.join(packagePath, "testing/pointcloud_1.npy"))

    # saved_pc += np.random.normal(0, noiseVar, saved_pc.shape)
    cloud = pcl.PointCloud_PointXYZI()
    cloud.from_array(saved_pc)

    start = time.time()
    lidarPipeline.setPointcloud(cloud)
    output = lidarPipeline.run()
    end = time.time()

    gt_detected = np.load(os.path.join(packagePath, "testing/pointcloud_1_gt.npy"))

    dists = []
    for cone in gt_detected:
        dist = np.linalg.norm(output["detected"] - cone.reshape(1, -1), axis=1)
        min_dist = np.min(dist)
        dists.append(min_dist)
        # self.assertTrue(min_dist < 0.1)

    total_time = (end - start) * 1000
    # self.assertLess(total_time, 30)
    # self.assertTrue(len(output['detected']) == len(gt_detected))

    if REPORT:
        print("Pipeline took {} ms".format(total_time))
        print("Mean distance to GT: {}".format(np.mean(dists)))
        print("Std distance to GT: {}".format(np.std(dists)))
        print("Detected {} cones".format(len(output["detected"])))
        print("Ground truth {} cones".format(len(gt_detected)))
        plt.scatter(gt_detected[:, 0], gt_detected[:, 1])
        plt.scatter(output["detected"][:, 0], output["detected"][:, 1])
        # plt.scatter(output['cluster_centers'][:, 0], output['cluster_centers'][:, 1])
        plt.show()


if __name__ == "__main__":
    testEasy()
