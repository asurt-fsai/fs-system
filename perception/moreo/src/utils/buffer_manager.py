# mypy: allow-any-unimported
"""
This class, "BufferManager" is used to store a limited number of
readings, i.e. image messages, odometry, and bounding boxes.
It allows you to add new readings to the buffer, get the next and previous index in the buffer,
and get a pair of readings based on a specified baseline distance. It uses the feature extractor
to extract features from the images and bounding boxes.
"""
import typing
import numpy.typing as npt
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from .reading import BaseReading, Reading  # pylint: disable=import-error
from .oldDistributedFEX import (  # pylint: disable=import-error
    DistributedFeatureDetectionSystem,
)


class BufferManager:
    """
    The BufferManager class is responsible for managing a buffer of readings,
    including adding new readings to the buffer and
    retrieving pairs of readings based on a specified baseline distance.
    The __init__ method initializes the buffer manager object with
    the specified buffer size and feature extractor. The buffer is initially
    filled with empty BaseReading objects, and the available baselines
    between readings are set to an array of 0s with the same length as the buffer size.
    The getNextIdx and getPreviousIdx methods return the next and
    previous indices in the buffer, respectively, wrapping around
    to the start or end of the buffer if necessary.
    The addToBuffer method adds an image message, odometry,
    and bounding boxes to the buffer, creating a new Reading
    object and updating the available baselines between readings.
    The getPair method returns a pair of readings from the
    buffer based on the specified baseline distance, and
    also calls the feature extractor to extract features
    from the images and bounding boxes of the current reading.
    """

    def __init__(
        self, bufferSize: int, featureExtractor: DistributedFeatureDetectionSystem
    ) -> None:
        """
        Initialize the buffer manager object with the specified buffer size and feature extractor.

        :param bufferSize: The maximum number of items that can be stored in the buffer.
        :param featureExtractor: An object that can extract features from images and bounding boxes.
        """
        self.bufferSize = bufferSize
        self.buffer = []
        emptyReading = BaseReading()
        for _ in range(bufferSize):
            self.buffer.append(emptyReading)
        self.availableBaselines = np.asarray([0] * self.bufferSize)
        self.currentIdx = 0
        self.featureExtractor = featureExtractor

    def getNextIdx(self) -> int:
        """
        Get the next index in the buffer, wrapping around to the start if necessary.

        :return: The next index in the buffer.
        """
        return (int)((self.currentIdx + 1) % self.bufferSize)

    def getPreviousIdx(self) -> int:
        """
        Get the previous index in the buffer, wrapping around to the end if necessary.

        :return: The previous index in the buffer.
        """
        return (int)((self.currentIdx - 1) % self.bufferSize)

    def addToBuffer(self, imgMsg: Image, odom: Odometry, bboxes: npt.NDArray[np.float64]) -> None:
        """
        Add an image message, odometry, and bounding boxes to the buffer.

        :param imgMsg: The image message to be added to the buffer.
        :param odom: The odometry of the image to be added to the buffer.
        :param bboxes: The bounding boxes associated with the image to be added to the buffer.
        """
        currentReading = Reading(imgMsg, odom, bboxes)
        previousReading = self.buffer[self.getPreviousIdx()]
        if isinstance(previousReading, Reading):
            distanceFromPreviousReading = np.sqrt(
                np.sum(
                    np.power(
                        (currentReading.getPosition() - previousReading.getPosition()),
                        2,
                    )
                )
            )
            self.availableBaselines = self.availableBaselines + distanceFromPreviousReading

        self.buffer[self.currentIdx] = currentReading
        self.availableBaselines[self.currentIdx] = 0
        self.currentIdx = self.getNextIdx()

    def getPair(self, baseline: float) -> typing.Tuple[BaseReading, BaseReading]:
        """
        Get a pair of readings from the buffer based on the specified baseline.

        :param baseline: The desired baseline distance between the two readings.
        :return: A pair of readings from the buffer where the destince between the two readings
        is approximatley equal to the passed baseline.
        """

        currentReading = self.buffer[self.getPreviousIdx()]
        if isinstance(currentReading, Reading):
            features = self.featureExtractor.getFeatures(
                currentReading.getImage(), currentReading.getBboxes()
            )
            featuresInBbox = {}
            for idx, box in enumerate(currentReading.getBboxes()):
                _, _, _, _, bboxId, _ = box
                kps = features[idx][0]
                des = np.asarray(features[idx][1])
                featuresInBbox[str(bboxId)] = (box, kps, des)

            currentReading.setFeaturesPerBbox(featuresInBbox)

            idxOfPrev = np.argsort(np.abs(self.availableBaselines - baseline))[0]
            # print(np.argsort(np.abs(self.availableBaselines-baseline)),"
            # With distances:",np.abs(self.availableBaselines-baseline))
            if idxOfPrev == self.getPreviousIdx():
                idxOfPrev = np.argsort(np.abs(self.availableBaselines - baseline))[1]
                # print("Retunred baseline was:",
                # self.calc_baseline(self.buffer[idxOfPrev],currentReading))

            previousReading = self.buffer[idxOfPrev]

            if isinstance(previousReading, Reading):

                if len(previousReading.getFeaturesPerBbox().keys()) == 0:
                    features = self.featureExtractor.getFeatures(
                        previousReading.getImage(), previousReading.getBboxes()
                    )

                    featuresInBbox = {}
                    for idx, box in enumerate(previousReading.getBboxes()):
                        _, _, _, _, bboxId, _ = box
                        kps = features[idx][0]
                        des = np.asarray(features[idx][1])
                        featuresInBbox[str(bboxId)] = (box, kps, des)
                    previousReading.setFeaturesPerBbox(featuresInBbox)
            return previousReading, currentReading

        return BaseReading(), BaseReading()

    def calcBaseline(self, previousReading: Reading, currentReading: Reading) -> float:
        """
        Calculate the baseline distance between two readings in the buffer.

        :param reading1: The first reading to use for calculating the baseline distance.
        :param reading2: The second reading to use for calculating the baseline distance.
        :return: The baseline distance between the two readings.
        """
        if isinstance(previousReading, Reading) and isinstance(currentReading, Reading):
            relativeTranslation = (
                currentReading.getPosition() - previousReading.getPosition()
            ).reshape(3, 1)
            return (float)(np.sqrt(np.sum(np.power(relativeTranslation, 2))))
        return 0
