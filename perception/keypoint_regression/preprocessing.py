"""Extracts bounding boxes from the image and preprocesses them for the model.
"""
# pylint: disable=too-few-public-methods
from typing import List, Optional, Tuple

import torch

from torchvision import transforms

import numpy as np

import numpy.typing as npt


class ImagePreprocessing:
    """This class is used to preprocess the image and the bounding boxes"""

    def _extractBoundingBoxes(
        self, img: torch.Tensor, bboxes: List[List[int]]
    ) -> Tuple[List[torch.Tensor], torch.Tensor, torch.Tensor]:
        """Crop the image according to the bounding boxes.
        Parameters:
        -----------
            img: np.ndarray
                Image to be cropped.
            bboxes: np.ndarray
                Bounding boxes to crop the image.
        """
        bbox: Optional[torch.Tensor] = None
        bboxesImages = []
        topLeft = []
        originalDim = []

        for box in bboxes:
            topLeftX, topLeftY = box[0], box[1]
            bottomRightX, bottomRightY = box[2], box[3]
            bbox = img[topLeftY:bottomRightY, topLeftX:bottomRightX]

            topLeft.append((topLeftX, topLeftY))
            originalDim.append((bbox.shape[1], bbox.shape[0]))
            bboxesImages.append(bbox)

        boundingBoxTransformationMatrix = torch.from_numpy(np.array(topLeft))
        originalBboxesDimsMatrix = torch.from_numpy(np.array(originalDim))

        return bboxesImages, boundingBoxTransformationMatrix, originalBboxesDimsMatrix

    def _resizeNormalizeBboxes(
        self, bboxesImages: List[torch.Tensor], width: int, height: int
    ) -> torch.Tensor:
        """Resize and normalize the bounding boxes.
        Parameters:
        -----------
            bboxesImages: List[np.ndarray]
                List of bounding boxes images.
            width: int
                Width of the resized image.
            height: int
                Height of the resized image.
        """
        transform = transforms.Compose(
            [
                transforms.Resize((width, height)),
                transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )
        resizedBboxes = bboxesImages[0].float().permute(2, 0, 1)
        resizedBboxes = transform(resizedBboxes / 255).unsqueeze(0)

        for img in bboxesImages[1:]:
            img = img.permute(2, 0, 1).float()
            img = transform(img / 255)
            resizedBboxes = torch.cat((resizedBboxes, img.unsqueeze(0)), dim=0)

        return resizedBboxes

    def runPreprocessingPipeline(
        self,
        img: npt.NDArray[np.float64],
        width: int,
        height: int,
        bboxes: List[List[int]],
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Preprocess the bounding boxes.
        Parameters:
        -----------
            img: tensor.Tensor
                Image to be preprocessed.
            width: int
                Width of the resized image.
            height: int
                Height of the resized image.
            bboxes: np.ndarray
                Bounding boxes to crop the image.
        Returns:
        --------
            resized_bboxes: torch.Tensor
                Tensor of normalized bounding box images.
        """
        imgTensor = torch.from_numpy(img)

        (
            bboxesImages,
            boundingBoxTransformationMatrix,
            originalBboxesDimsMatrix,
        ) = self._extractBoundingBoxes(imgTensor, bboxes)
        resizedNormalizedBBoxTensor = self._resizeNormalizeBboxes(bboxesImages, width, height)

        return (
            resizedNormalizedBBoxTensor,
            boundingBoxTransformationMatrix,
            originalBboxesDimsMatrix,
        )


# if __name__ == "__main__":
#     imagePreprocessor = ImagePreprocessing()
#     bboxes = torch.tensor(
#         [[1, 61, 171, 569], [75, 57, 241, 472], [155, 65, 266, 407], [224, 64, 335, 360]]
#     )
#     img = plt.imread("img.jpeg")
#     print(img.shape)
#     res = imagePreprocessor.runPreprocessingPipeline(img, 50, 80, bboxes)
#     print(res[0].shape, res[1].shape, res[2].shape)
