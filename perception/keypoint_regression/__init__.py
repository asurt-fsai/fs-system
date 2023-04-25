# mypy: ignore-errors
"""Fascade for keypoint regression."""

from typing import List

import numpy.typing as npt

import torch

from model import KeypointNet

from preprocessing import ImagePreprocessing

from postprocessing import PostProcessing


class KeypointsRegressionFascade:  # pylint: disable = too-few-public-methods

    """Fascade for keypoint regression."""

    IMAGE_WIDTH = 80  # pylint: disable=invalid-name
    IMAGE_HEIGHT = 80  # pylint: disable=invalid-name

    def __init__(self, modelPath: str):
        """
        Follows the Fascade design pattern to provide a single interface to the
        keypoint regression module.

        Parameters:
        -----------
            modelPath: str
                path of the model to be used for keypoint regression
        """
        # model
        self.modelPath = modelPath
        self.model = KeypointNet(inputChannels=1, outKeypoints=8)
        self.model.load_state_dict(torch.load(self.modelPath))
        self.model.eval()

        # preprocessing
        self.imagePreprocessing = ImagePreprocessing()

        # postprocessing
        self.postProcessing = PostProcessing()

    def run(self, img: npt.NDArray, bboxes: List[List[int]]):
        """
        Runs the keypoint regression module on the given image and bounding boxes

        Parameters:
        -----------
            img: np.ndarray
                image to run the keypoint regression on
            bboxes: List[List[int]]
                bounding boxes to run the keypoint regression on

        Returns:
        --------
            keypoints: List[List[int]]
                list of keypoints
        """
        # preprocessing
        (
            bboxesImages,
            boundingBoxTransformationMatrix,
            originalBboxesDimsMatrix,
        ) = self.imagePreprocessing.runPreprocessingPipeline(
            img, self.IMAGE_WIDTH, self.IMAGE_HEIGHT, bboxes
        )

        # model
        keypoints = self.model(bboxesImages.cuda())
        keypoints = keypoints.detach().cpu().numpy()

        # postprocessing
        keypoints = self.postProcessing(
            keypoints, originalBboxesDimsMatrix, boundingBoxTransformationMatrix
        )

        return keypoints
