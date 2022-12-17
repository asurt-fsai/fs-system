"""
Tests for the Serializers methods
"""
# pylint: disable=all
import os
import sys
import time

# Imports (UGLY Stuff)
packagePath, _ = os.path.split(os.path.split(__file__)[0])

for i in range(4):
    packagePath = os.path.split(packagePath)[0]

sys.path.insert(0, os.path.join(packagePath, "devel/lib/python3/dist-packages"))

from sensor_msgs.msg import PointCloud2
from asurt_msgs.msg import LandmarkArray

from src.modules.LidarPipeline import LidarPipeline
from src.modules.Filter.Filter import Filter
from src.modules.Filter.GroundRemoval import RansacGroundRemoval
from src.modules.ConeClassifier import ConeClassifier
from src.modules.Clusterer.AbstractClusterer import Clusterer

from src.ros.modules.Builders import (
    buildPipeline,
    buildTracker,
    buildFilter,
    buildConeClassifier,
    buildClusterer,
)  # pylint: disable=all

REPORTTIME = False


def testBuildFilter() -> None:
    """
    Tests the buildFilter method
    """
    tic = time.time()
    filterer = buildFilter()
    toc = time.time()

    if REPORTTIME:
        print(f"buildFilter took {(toc - tic)*1000}ms")
    assert isinstance(filterer, Filter)
    assert isinstance(filterer.groundRemovalMethod, RansacGroundRemoval)


def testBuildConeClassifier() -> None:
    """
    Tests the buildConeClassifier method
    """
    tic = time.time()
    coneClassifier = buildConeClassifier()
    toc = time.time()

    if REPORTTIME:
        print(f"buildConeClassifier took {(toc - tic)*1000}ms")
    assert isinstance(coneClassifier, ConeClassifier)


def testBuildClusterer() -> None:
    """
    Tests the buildClusterer method
    """
    tic = time.time()
    clusterer = buildClusterer()
    toc = time.time()

    if REPORTTIME:
        print(f"buildClusterer took {(toc - tic)*1000}ms")
    assert isinstance(clusterer, Clusterer)


def testBuildPipeline() -> None:
    """
    Tests the buildPipeline method
    """
    tic = time.time()
    lidarPipeline = buildPipeline()
    toc = time.time()

    if REPORTTIME:
        print(f"buildPipeline took {(toc - tic)*1000}ms")
    assert isinstance(lidarPipeline, LidarPipeline)
