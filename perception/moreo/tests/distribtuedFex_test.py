"""test file for the distributed_fex module
"""
# pylint: disable=all
import sys

import os

CURRENT_DIR = os.getcwd()
sys.path.append(os.path.join(CURRENT_DIR, "..", "src"))

import copy
import matplotlib.pyplot as plt

import cv2 as cv
import time
from DistributedFex import createMask, DistributedFeatureDetectionSystem


def getTimeDifferenceMS(snapshot1: int, snapshot2: int) -> float:
    """gets time difference in millisecond

    Parametes:
    ----------
        snapshot1: int
            first time snapshot
        snapshot2: int
            second time snapshot

    Returns:
        float
            time difference in milliseconds
    """
    return abs(snapshot2 - snapshot1) / 1e6


import numpy as np


def miniTest() -> None:
    """small unit test for distributed FEX"""
    boundingBoxes = [
        [1, 61, 171, 569],
        [75, 57, 241, 472],
        [155, 65, 266, 407],
        [75, 57, 241, 472],
        [224, 64, 335, 360],
        [286, 48, 391, 310],
        [378, 38, 443, 251],
    ]

    image = plt.imread(os.path.join(CURRENT_DIR, "121.jpg"))

    # feature extraction without distributed fex
    snapshot1 = time.time_ns()
    for boundingBox in boundingBoxes:
        mask = createMask(image.shape, boundingBox)
        orb = cv.ORB_create()
        keypoint = orb.detect(image, mask)
        keypoint, _ = orb.compute(image, keypoint)

    snapshot2 = time.time_ns()
    print(
        "Execution Time: ",
        getTimeDifferenceMS(snapshot1, snapshot2),
        "ms",
        f"for {len(boundingBoxes)} boundingBoxes and image size {image.shape}"
        + "without distributedFEX",
    )

    # feature extraction with distributed fex
    snapshot1 = time.time_ns()
    moreoFEX = DistributedFeatureDetectionSystem()
    # ⚠ @NOTE set visualization to false when testing execution time
    keypointsList, descriptorsList = moreoFEX.getFeatures(image, boundingBoxes, visualize=False)
    snapshot2 = time.time_ns()
    print(
        "Execution Time: ",
        getTimeDifferenceMS(snapshot1, snapshot2),
        "ms",
        f"for {len(boundingBoxes)} boundingBoxes and image size {image.shape} with distributedFEX",
    )
    print("results", len(descriptorsList), len(keypointsList))


if __name__ == "__main__":
    miniTest()
