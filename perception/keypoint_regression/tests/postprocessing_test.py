"""This class is used to test the image postprocessing class methods"""
# pylint: disable=all
import sys

import unittest

sys.path.append("..")


import numpy as np

import torch

import matplotlib.pyplot as plt

from postprocessing import PostProcessor


class TestPostProcessor(unittest.TestCase):
    """
    A class for testing the ImagePreprocessing class methods.

    Attributes:
    -----------
    img_preprocessing : ImagePreprocessing
        An instance of ImagePreprocessing class.
    image :torch.Tensor
        A test image.
    """

    def setUp(self) -> None:
        """
        A method to set up the ImagePreprocessing instance and test image path
        before each test method is called.
        """
        self.postprocessor = PostProcessor()
        self.image = plt.imread("121.jpeg")

    def test_postProcessingStep(self) -> None:
        """
        A method to test the postProcessingStep() method of the PostProcessor class.
        """
        keyPointArray = torch.tensor([[0.5, 0.5], [0.5, 0.5]])
        imageSize = torch.tensor([224, 224])
        topLeftBbox = torch.tensor([50, 50])
        res = self.postprocessor.postProcessingStep(keyPointArray, imageSize, topLeftBbox)
        self.assertEqual(res.shape, (2, 2))
        self.assertEqual(res[0][0], 162.0)
        self.assertEqual(res[0][1], 162.0)
