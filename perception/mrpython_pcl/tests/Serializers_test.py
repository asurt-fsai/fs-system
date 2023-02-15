"""
Tests for the Serializers methods
"""
# pylint: disable=all
import os
import sys
import time

import pcl
import numpy as np

# Imports (UGLY Stuff)
packagePath, _ = os.path.split(os.path.split(__file__)[0])
path = os.path.join(packagePath, "src")
sys.path.insert(0, path)

for i in range(4):
    packagePath = os.path.split(packagePath)[0]

sys.path.insert(0, os.path.join(packagePath, "devel/lib/python3/dist-packages"))

from sensor_msgs.msg import PointCloud2
from asurt_msgs.msg import LandmarkArray

from mrpython_pcl.ros.Serializers import (
    rosToPcl,
    npToPcl,
    pclToRos,
    npToRos,
    npConesToRos,
)

REPORTTIME = False


def testPclToRosToPcl() -> None:
    points = np.random.normal(0, 5, (5000, 3))

    cloud = pcl.PointCloud()
    cloud.from_array(points.astype(np.float32))

    rosPcl = pclToRos(cloud)
    assert isinstance(rosPcl, PointCloud2)

    cloudBack = rosToPcl(rosPcl)
    assert isinstance(cloudBack, pcl.PointCloud)

    pointsBack = cloudBack.to_array()

    # Check if points and pointsBack are the same, regardless of order
    assert np.allclose(np.sort(points, axis=0), np.sort(pointsBack, axis=0), atol=1e-3)


def testNpToPclToNp() -> None:
    points = np.random.normal(0, 5, (5000, 3))

    cloudArray = npToPcl(points)
    assert isinstance(cloudArray, pcl.PointCloud)

    pointsBack = cloudArray.to_array()

    # Check if points and pointsBack are the same, regardless of order
    assert np.allclose(np.sort(points, axis=0), np.sort(pointsBack, axis=0), atol=1e-3)


def testNpToRosToNp() -> None:
    points = np.random.normal(0, 5, (5000, 3))

    rosPcl = npToRos(points)
    assert isinstance(rosPcl, PointCloud2)

    pointsBack = rosToPcl(rosPcl).to_array()

    # Check if points and pointsBack are the same, regardless of order
    assert np.allclose(np.sort(points, axis=0), np.sort(pointsBack, axis=0), atol=1e-3)


def testNpConesToRos() -> None:
    # TODO: test adding ids for cones
    cones = np.random.normal(0, 5, (5000, 2))

    landmarks = npConesToRos(cones)
    assert isinstance(landmarks, LandmarkArray)

    conesBack = []
    for landmark in landmarks.landmarks:
        conesBack.append(landmark.position.x)
        conesBack.append(landmark.position.y)

    conesBack = np.array(conesBack).reshape(-1, 2)

    # Check if cones and conesBack are the same, regardless of order
    assert np.allclose(np.sort(cones, axis=0), np.sort(conesBack, axis=0), atol=1e-3)


def testTime() -> None:
    points = np.random.normal(0, 5, (5000, 3))

    tic = time.time()
    rosPc = npToRos(points)
    toc = time.time()
    npToRosTime = (toc - tic) * 1000

    tic = time.time()
    cloud = rosToPcl(rosPc)
    toc = time.time()
    rosToPclTime = (toc - tic) * 1000

    tic = time.time()
    cloudFromNp = npToPcl(points)
    toc = time.time()
    npToPclTime = (toc - tic) * 1000

    tic = time.time()
    rosPcFromPcl = pclToRos(cloud)
    toc = time.time()
    pclToRosTime = (toc - tic) * 1000

    tic = time.time()
    landmarkMsg = npConesToRos(points[:, :2])
    toc = time.time()
    npConesToRosTime = (toc - tic) * 1000

    if REPORTTIME:
        print("npToRosTime: ", npToRosTime)
        print("rosToPclTime: ", rosToPclTime)
        print("npToPclTime: ", npToPclTime)
        print("pclToRosTime: ", pclToRosTime)
        print("npConesToRosTime: ", npConesToRosTime)
