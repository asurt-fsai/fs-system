"""test file for model.py"""
# pylint: disable=all
import sys

import unittest

import datetime

import torch

sys.path.append("..")

from model import KeypointNet

from dataset import KeypointDataset


class TesTModel(unittest.TestCase):
    """
    A class for testing the model class methods.

    Attributes:
    -----------
    model : KeypointNet
        An instance of KeypointNet class.
    dataset : KeypointDataset
        An instance of KeypointDataset class.
    """

    def setUp(self) -> None:
        """
        A method to set up the KeypointNet instance and KeypointDataset instance
        before each test method is called.
        """
        self.model = KeypointNet(1, 8)
        self.dataset = KeypointDataset("../data/imgs", "../data/json")

    def test_inferenceTime(self) -> None:
        """
        A method to test the inferenceTime() method of the KeypointNet class.

        Asserts:
        --------
        - The average inference time is less than 10ms.
        """
        total = 0.0
        ITERATIONS = 100
        self.model.eval()

        for _ in range(ITERATIONS):
            imagesBatch = torch.randn(80, 1, 80, 80)  # pylint: disable=no-member
            time1 = datetime.datetime.now()

            # put images on cuda
            imagesBatch = imagesBatch.cuda()
            _ = self.model(imagesBatch)

            # computing time difference and calculating average execution time
            time2 = datetime.datetime.now()
            total += (time2 - time1).microseconds * 1e-3

        self.assertLess(total / ITERATIONS, 10)

    def test_modelOutput(self) -> None:
        """
        A method to test the modelOutput() method of the KeypointNet class.

        Asserts:
        --------
        - The output tensor has the expected shape.
        """
        imagesBatch = torch.randn(80, 1, 80, 80)
        self.model = self.model.cuda()
        imagesBatch = imagesBatch.cuda()
        output = self.model(imagesBatch)
        self.assertEqual(output.shape, (80, 8))
