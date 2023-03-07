"""This class is used to test the ImagePreprocessing class methods"""
import unittest
import numpy as np
import torch
from image_preprocessing import ImagePreprocessing


class TestImagePreprocessing(unittest.TestCase):
    """
    A class for testing the ImagePreprocessing class methods.

    Attributes:
    -----------
    img_preprocessing : ImagePreprocessing
        An instance of ImagePreprocessing class.
    img_path : str
        The path to the test image file.
    """

    def setUp(self):
        """
        A method to set up the ImagePreprocessing instance and test image path
        before each test method is called.
        """
        self.img_preprocessing = ImagePreprocessing()
        self.img_path = "121.jpeg"

    def test_load_image(self):
        """
        A method to test the load_image() method of the ImagePreprocessing class.

        Asserts:
        --------
        - The loaded image is an instance of the torch.Tensor class.
        """
        image = self.img_preprocessing.load_image(self.img_path)
        self.assertIsInstance(image, torch.Tensor)

    def test_crop_image(self):
        """
        A method to test the crop_image() method of the ImagePreprocessing class.

        Asserts:
        --------
        - The number of cropped images is equal to the number of input bounding boxes.
        - The length of top_left and original_dim lists is equal to the number of input.
        """
        img = self.img_preprocessing.load_image(self.img_path)
        bboxes = [[50, 50, 300, 300], [100, 100, 400, 400]]
        self.img_preprocessing.crop_image(img, bboxes)
        self.assertEqual(len(self.img_preprocessing.bboxes_images), len(bboxes))
        self.assertEqual(len(self.img_preprocessing.top_left), len(bboxes))
        self.assertEqual(len(self.img_preprocessing.original_dim), len(bboxes))

    def test_preprocess_bboxes(self):
        """
        A method to test the preprocess_bboxes() method of the ImagePreprocessing class.

        Asserts:
        --------
        - The preprocessed bounding boxes tensor has the expected shape.
        """
        img = np.ones((500, 500, 3), dtype=np.uint8) * 255
        bboxes = [[50, 50, 300, 300], [100, 100, 400, 400]]
        width, height = 224, 224
        norm_bboxes = self.img_preprocessing.preprocess_bboxes(
            img, width, height, bboxes
        )
        self.assertIsInstance(norm_bboxes, torch.Tensor)
        self.assertEqual(norm_bboxes.shape, (len(bboxes), 3, 224, 224))


if __name__ == "__main__":
    unittest.main()
