"""this module provides opencv like API for distributed
feature extraction using a process pool
"""

import multiprocessing
from multiprocessing.managers import ListProxy

from typing import List, Tuple

import numpy as np

import matplotlib.pyplot as plt

from cv2 import (  # pylint: disable=no-name-in-module
    rectangle,
    cvtColor,
    drawKeypoints,
    ORB_create,
    COLOR_BGR2GRAY,
)


class DistributedFeatureDetectionSystem:
    """distributed feature extraction module"""

    def __init__(self) -> None:
        """
        Constructor of the distributed feature detector system.
        It uses orb to detect features but in a distributed fashion

        Parameters:
        -----------
        nFeatures: int Default=1000
            max threshold of the orb features
        """
        self.pool = multiprocessing.Pool(processes=5)  # pylint: disable=consider-using-with

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
        man = multiprocessing.Manager()
        keypointsList = man.list()
        descriptorsList = man.list()

        # loop over all boundingBoxes

        args = (
            (image.copy(), boundingBoxes[i], keypointsList, descriptorsList)
            for i in range(len(boundingBoxes))
        )
        res = self.pool.starmap_async(createFeatureExtractionTask, args)
        res.wait()

        if visualize:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 10))
            fig.suptitle("Moreo Results: boundingBoxes & Extracted Features")

            visualizableImage = image.copy()
            for i, boundingBox in enumerate(boundingBoxes):
                # bounding box
                rectangle(  # pylint: disable=c-extension-no-member
                    visualizableImage,
                    (boundingBox[0], boundingBox[1]),
                    (boundingBox[2], boundingBox[3]),
                    255,
                    2,
                )
                # draw keypoints
                visualizableImage = drawKeypoints(  # pylint: disable=c-extension-no-member
                    visualizableImage, keypointsList[i], None, color=(0, 255, 0)
                )

            ax1.set_title("Before")
            ax2.set_title("After")
            ax1.imshow(image)
            ax2.imshow(visualizableImage)
            plt.show()

        return keypointsList, descriptorsList

    def terminateProcesses(self) -> None:
        """terminates process pool processes when desired"""
        self.pool.terminate()


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
    mask = cvtColor(mask.copy(), COLOR_BGR2GRAY)
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

    orb = ORB_create(nfeatures=1000)  # pylint: disable=c-extension-no-member

    # orb feature extraction step
    keypoint = orb.detect(image, mask)
    keypoint, des = orb.compute(image, keypoint)
    keypoint = [(kp.pt, kp.size, kp.angle, kp.response, kp.octave, kp.class_id) for kp in keypoint]
    keypointsList.append(keypoint)

    descriptorsList.append(des)
