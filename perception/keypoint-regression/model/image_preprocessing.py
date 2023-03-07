"""
This module provides functions for image preprocessing .
"""
from typing import List

import torch

import numpy as np

import matplotlib.pyplot as plt

import torchvision.transforms as transforms


class ImagePreprocessing:

    """This class is used to preprocess the image and the bounding boxes"""

    def __init__(self):

        """Constructor for the class
        parameters:
        -----------
        bboxes_images: list

             a list of cropped bounding box images.

        top_left :torch.Tensor

             a tensor of top-left corner coordinates of the bounding boxes.

        original_dim : torch.Tensor

             a tensor of original dimensions (width, height) of the bounding boxes.

        normalize_bboxes :torch.Tensor

             a tensor of normalized bounding box images.

        """

        self.bboxes_images = None

        self.top_left = None

        self.original_dim = None

    def load_image(self, img_path: str):

        """Load an image from a file as a numpy array.

        Parameters:
        -----------
            img_path: str

                Path to the image file.

        Returns:
        --------

            image: np.ndarray

                Loaded image as a numpy array.

        """

        image = plt.imread(img_path)
        image = torch.from_numpy(np.array(image))  # pylint: disable=no-member
        return image

    def crop_image(self, img: torch.Tensor, bboxes: List[List[int]]):

        """Crop the image according to the bounding boxes.

        Parameters:
        -----------

            img: torch.Tensor

                Image to be cropped.

            bboxes: np.ndarray

                Bounding boxes to crop the image.

        """

        bbox = []

        bboxes_images = []

        top_left = []

        original_dim = []
        for box in bboxes:

            x_1, y_1 = box[0], box[1]

            x_2, y_2 = box[2], box[3]

            top_left.append((x_1, y_1))

            bbox = img[y_1:y_2, x_1:x_2]

            original_dim.append((bbox.shape[1], bbox.shape[0]))

            bboxes_images.append(bbox)

        self.top_left = torch.from_numpy(np.array(top_left))  # pylint: disable=no-member

        self.original_dim = torch.from_numpy(np.array(original_dim)) # pylint: disable=no-member

        self.bboxes_images = bboxes_images

    def _resize_normalize_bboxes(self, width: int, height: int):

        """Resize and normalize the bounding boxes.

        Parameters:
        -----------

            width: int

                Width of the resized image.
            height: int

                Height of the resized image.

         Returns:
        --------

            resized_normilze_bboxes: torch.Tensor

                Tensor of normalized bounding box images.

        """

        transform = transforms.Compose(
            [
                transforms.Resize((width, height)),
                transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )

        resized_normilze_bboxes = self.bboxes_images[0].float().permute(2, 0, 1)

        resized_normilze_bboxes = transform(resized_normilze_bboxes / 255).unsqueeze(0)

        for img in self.bboxes_images[1:]:

            img = img.permute(2, 0, 1).float()

            img = transform(img / 255)

            resized_normilze_bboxes = torch.cat((resized_normilze_bboxes, img.unsqueeze(0)), dim=0)  # pylint: disable=no-member

        return resized_normilze_bboxes

    def preprocess_bboxes(
        self, img: np.ndarray, width: int, height: int, bboxes: List[List[int]]
    ):

        """Preprocess the bounding boxes.

        Parameters:
        -----------

            img: np.ndarray

                Image to be preprocess.

            width: int

                Width of the resized image.
            height: int

                Height of the resized image.

            bboxes: np.ndarray

                Bounding boxes to crop the image.

        Returns:
        --------

            resized_normilze_bboxes: torch.Tensor

                Tensor of normalized bounding box images.

        """

        img = torch.from_numpy(img)  # pylint: disable=no-member

        self.crop_image(img, bboxes)
        return self._resize_normalize_bboxes(width, height)
