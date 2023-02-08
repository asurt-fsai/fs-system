""" this file provdides the implementation for keypoint regression
"""
# mypy: ignore-errors
from typing import Union, Tuple

import datetime

# pylint: disable=too-few-public-methods
import torch  # pylint: disable=import-error
from torch import nn  # pylint: disable=import-error


class KeypointNet(nn.Module):
    """keypoint regression class"""

    def __init__(self, outKeypoints: int = 8):
        """constructor

        Parameters:
        -----------
            outKeypoints: Default=8.
        """
        super().__init__()

        self.outKeypoints = outKeypoints
        self.convolutionalNet = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=(7, 5), stride=2, dilation=(4, 2)),
            nn.ELU(),
            nn.BatchNorm2d(64),
            ResBlock(dims=64, kernelSize=(3, 3)),
            nn.Conv2d(64, 128, kernel_size=(3, 3), stride=2, dilation=(4, 2)),
            nn.ELU(),
            nn.BatchNorm2d(128),
            ResBlock(dims=128, kernelSize=(3, 3)),
            nn.Conv2d(128, 256, kernel_size=(3, 3), dilation=(3, 2)),
            nn.ELU(),
            nn.BatchNorm2d(256),
            ResBlock(dims=256, kernelSize=(3, 3)),
            nn.Conv2d(256, 1, kernel_size=(3, 3)),
            nn.ELU(),
        )
        self.fcNet = nn.Linear(12 * 13, self.outKeypoints * 2)

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        """performs forward propagation

        Parameters:
        -----------
        img: torch.Tensor
            input bound image for the model to extract
            features from. The image input size is expected to be (100,50)

        Returns:
        --------
        torch.Tensor
            shape = (batch_size, self.outKeypoints) representing the keypoints
                of the cone in the input image relative to the image frame
        """
        img = self.convolutionalNet(img)
        batchSize, *_ = img.shape
        img = img.view(batchSize, -1)
        img = self.fcNet(img)
        img = nn.functional.relu(img)

        return img


class ResBlock(nn.Module):
    """Represents Residual blocks from resenet paper:
    Deep Residual Learning for Image Recognition  https://arxiv.org/abs/1512.03385
    """

    def __init__(self, dims: int, kernelSize: Union[int, Tuple[int]] = 3):
        """constructor

        Parameters:
        -----------
        dims:(int, Tuple)
            input and output dimension
        kernelSize: Union[int, Tuple[int]] default=3
            convolution kernel size
        """
        super().__init__()
        if isinstance(kernelSize, Tuple):
            padding = (int(kernelSize[0] // 2) * 2, int(kernelSize[1] // 2) * 2)
        else:
            padding = int(kernelSize // 2) * 2

        self.model = nn.Sequential(
            nn.Conv2d(dims, dims, kernel_size=kernelSize, padding=padding, stride=1),
            nn.ReLU(),
            nn.Conv2d(dims, dims, kernel_size=kernelSize, padding=padding, stride=1),
            nn.ReLU(),
        )

    def forward(self, featureVector: torch.Tensor) -> torch.Tensor:
        """performs forward propagation

        Parameters:
        -----------
        featureVector: torch.Tensor
            input feature vector for resblock

        Returns:
        --------
        torch.Tensor
            represents the features vector output
            of the resblock
        """
        return self.model(featureVector)


def miniTest():
    """tests little tests to the module"""
    model = KeypointNet()
    model = model.cuda()

    total = 0
    iterations = 1000
    model.eval()

    for _ in range(iterations):

        imagesBatch = torch.randn(40, 3, 80, 50)  # pylint: disable=no-member
        time1 = datetime.datetime.now()

        # put images on cuda
        imagesBatch = imagesBatch.cuda()
        _ = model(imagesBatch)

        # computing time difference and calculating average execution time
        time2 = datetime.datetime.now()
        total += (time2 - time1).microseconds / 1000

    print(total / iterations, "ms")


if __name__ == "__main__":
    miniTest()
