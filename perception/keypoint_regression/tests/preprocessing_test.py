"""This class is used to test the ImagePreprocessing class methods"""
# pylint: disable=all
import sys

import unittest

sys.path.append("..")


import numpy as np

import torch

import matplotlib.pyplot as plt

from preprocessing import ImagePreprocessing


class TestImagePreprocessing(unittest.TestCase):
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
        self.img_preprocessing = ImagePreprocessing()
        self.image = plt.imread("121.jpeg")

    def test_preprocessBboxes(self) -> None:
        """
        A method to test the preprocess_bboxes() method of the ImagePreprocessing class.

        Asserts:
        --------
        - The number of bboxes images is equal to the number of input bounding boxes.
        - The length of bounding box transformation matrix  and original bboxes dims matrix lists is equal to the number of input.
        - The preprocessed bounding boxes tensor has the expected shape.
        """
        bboxes = [[50, 50, 300, 300], [100, 100, 400, 400]]
        width, height = 224, 224
        (
            bboxesImage,
            boundingBoxTransformationMatrix,
            originalBboxesDimsMatrix,
        ) = self.img_preprocessing.runPreprocessingPipeline(self.image, width, height, bboxes)
        self.assertEqual(len(boundingBoxTransformationMatrix), len(bboxes))
        self.assertEqual(len(originalBboxesDimsMatrix), len(bboxes))
        self.assertIsInstance(bboxesImage, torch.Tensor)
        self.assertEqual(bboxesImage.shape, (len(bboxes), 3, 224, 224))


if __name__ == "__main__":
    unittest.main()
