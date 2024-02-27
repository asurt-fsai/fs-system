"""
Tests for the Serializers methods
"""
# pylint: disable=all
import time
import pytest

from mrpython_pcl.LidarPipeline.LidarPipeline import LidarPipeline
from mrpython_pcl.LidarPipeline.Filter.Filter import Filter
from mrpython_pcl.LidarPipeline.GroundRemoval.SimpleGroundRemoval import SimpleGroundRemoval
from mrpython_pcl.LidarPipeline.ConeClassifier import ConeClassifier
from mrpython_pcl.LidarPipeline.Clusterer.AbstractClusterer import Clusterer

from mrpython_pcl.ros.Builders import Builder

REPORTTIME = False


def testBuildFilter() -> None:
    """
    Tests the buildFilter method
    """
    builder = Builder(True)
    tic = time.time()
    filterer = builder.buildFilter()
    toc = time.time()

    if REPORTTIME:
        print(f"buildFilter took {(toc - tic)*1000}ms")
    assert isinstance(filterer, Filter)
    assert isinstance(filterer.groundRemovalMethod, SimpleGroundRemoval)


def testBuildConeClassifier() -> None:
    """
    Tests the buildConeClassifier method
    """
    builder = Builder(True)
    tic = time.time()
    coneClassifier = builder.buildConeClassifier()
    toc = time.time()

    if REPORTTIME:
        print(f"buildConeClassifier took {(toc - tic)*1000}ms")
    assert isinstance(coneClassifier, ConeClassifier)


def testBuildClusterer() -> None:
    """
    Tests the buildClusterer method
    """
    builder = Builder(True)
    tic = time.time()
    clusterer = builder.buildClusterer()
    toc = time.time()

    if REPORTTIME:
        print(f"buildClusterer took {(toc - tic)*1000}ms")
    assert isinstance(clusterer, Clusterer)


def testBuildPipeline() -> None:
    """
    Tests the buildPipeline method
    """
    builder = Builder(True)
    tic = time.time()
    lidarPipeline = builder.buildPipeline()
    toc = time.time()

    if REPORTTIME:
        print(f"buildPipeline took {(toc - tic)*1000}ms")
    assert isinstance(lidarPipeline, LidarPipeline)


def testSecuringParamServer() -> None:
    """
    Ensures that using default value for parameters can be disabled
    and if enabled, works as expected
    """
    # Default values disabled
    builder = Builder(False)
    with pytest.raises(KeyError):
        builder.getParam("Doesn't exist", 1)

    # Default values enabled
    builder = Builder(True)
    defaultValue = builder.getParam("Doesn't exist", 1)
    assert defaultValue == 1

    with pytest.raises(KeyError):
        builder.getParam("Doesn't exist")
