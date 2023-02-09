"""this module provides opencv like API for distributed
feature extraction using a process pool
"""
import time

import multiprocessing
from multiprocessing.managers import ListProxy

from typing import List, Tuple

import numpy as np

import matplotlib.pyplot as plt

import cv2 as cv


def createMask(imageShape: Tuple[int], boundingBox: List[int]) -> np.ndarray:
    """
    creates a mask to mask the image and allow feature extraction
    only to the selected areas

    Parameters:
    -----------
    imageShape: Tuple[int]
        shape of the input image
    boundingBox: Tuple[int]
        bounding in the form of [x1, y1, x2, y2]

    Returns:
        np.array
            the mask of the region around the boundingBox
    """
    mask = np.zeros(imageShape, np.uint8)
    mask[boundingBox[1] : boundingBox[3], boundingBox[0] : boundingBox[2]] = 255
    return mask


def createFeatureExtractionTask(
    image: np.ndarray,
    boundingBox: List[int],
    keypointsList: List[float],
    descriptorsList: List[float],
) -> None:
    """
    task to extract the features of a single bounding box from the image,
    used with threads to distribute the process of extracting the bounding boxes

    Parameters:
    ----------
    image: np.array
        input image
    boundingBox: List[int]
        bounding boxes in the form of [x1, y1, x2, y2]
    keypointsList: List[float]
        keypoints results array
    descriptorsList: List[float]
        descriptors results array
    """
    imageShape = image.shape

    # creating mask step
    mask = createMask(imageShape, boundingBox)
    orb = cv.ORB_create(nfeatures=100)  # pylint: disable=c-extension-no-member

    # orb feature extraction step
    keypoint = orb.detect(image, mask)
    keypoint, des = orb.compute(image, keypoint)
    keypointsList.append(keypoint)
    descriptorsList.append(des)


class DistributedFeatureDetectionSystem:
    """_summary_"""

    def __init__(self) -> None:
        """
        Constructor of the distributed feature detector system.
        It uses orb to detect features but in a distributed fashion

        Parameters:
        -----------
        nFeatures: int Default=1000
            max threshold of the orb features
        """
        self.pool = multiprocessing.Pool(processes=5)

    def getFeatures(
        self, image: np.ndarray, boundingBoxes: List[List[int]], visualize: bool = False
    ) -> Tuple[ListProxy, ListProxy]:  # type: ignore
        """
        finds the features in the boundingBoxes of the image

        Parameters:
        -----------
        image: np.array
            input image
        boundingBoxes: List[List[int]]
            bounding boxes
        visualize: bool Default=False
            visualizes the feature extraction process using matplotlib

        Returns:
        --------
        Tuple[ListProxy[Any], ListProxy[Any]]
            tuple with keypoints at index=0 and descriptors at index=1

        """
        # results array
        nboundingBoxes = len(boundingBoxes)
        man = multiprocessing.Manager()
        keypointsList = man.list()
        descriptorsList = man.list()

        # loop over all boundingBoxes

        args = (
            (image, boundingBoxes[i], keypointsList, descriptorsList) for i in range(nboundingBoxes)
        )
        res = self.pool.starmap_async(createFeatureExtractionTask, args)
        res.wait()

        if visualize:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 10))
            fig.suptitle("Moreo Results: boundingBoxes & Extracted Features")

            visualizableImage = image.copy()
            for i in range(nboundingBoxes):
                boundingBox = boundingBoxes[i]
                # bounding box
                cv.rectangle(  # pylint: disable=c-extension-no-member
                    visualizableImage,
                    (boundingBox[0], boundingBox[1]),
                    (boundingBox[2], boundingBox[3]),
                    255,
                    2,
                )
                # draw keypoints
                keypoint = keypointsList[i]
                visualizableImage = cv.drawKeypoints(  # pylint: disable=c-extension-no-member
                    visualizableImage, keypoint, None, color=(0, 255, 0)
                )

            ax1.set_title("Before")
            ax2.set_title("After")
            ax1.imshow(image)
            ax2.imshow(visualizableImage)
            plt.show()

        return keypointsList, descriptorsList


####--- For Testing Purposes ---####


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


def miniTest() -> None:
    """small unit test for distributed FEX"""
    boundingBoxes = [
        [1, 61, 171, 569],
        [1, 61, 171, 569],
        [1, 61, 171, 569],
        [1, 61, 171, 569],
        [1, 61, 171, 569],
        [1, 61, 171, 569],
        [75, 57, 241, 472],
        [1, 61, 171, 569],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [75, 57, 241, 472],
        [155, 65, 266, 407],
        [75, 57, 241, 472],
        [224, 64, 335, 360],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [286, 48, 391, 310],
        [378, 38, 443, 251],
        [340, 38, 430, 273],
        [378, 38, 443, 251],
        [378, 38, 443, 251],
        [378, 38, 443, 251],
        [378, 38, 443, 251],
        [378, 38, 443, 251],
        [378, 38, 443, 251],
    ]

    image = plt.imread("121.jpg")

    # feature extraction without distributed fex
    snapshot1 = time.time_ns()
    for boundingBox in boundingBoxes:
        mask = createMask(image.shape, boundingBox)
        orb = cv.ORB_create(100)  # pylint: disable=c-extension-no-member
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
