""" this file provides the implementation for keypoint regression model
"""
# mypy: ignore-errors
from typing import Union, Tuple

# pylint: disable=too-few-public-methods
import torch  # pylint: disable=import-error
from torch import nn  # pylint: disable=import-error
from torch.nn.functional import sigmoid


class KeypointNet(nn.Module):
    """keypoint regression class"""

    def __init__(self, inputChannels, outKeypoints: int = 8):
        """constructor

        Parameters:
        -----------
            outKeypoints: Default=8.
        """
        super().__init__()
        self.outKeypoints = outKeypoints
        self.convPart = nn.Sequential(
            nn.Conv2d(inputChannels, 64, kernel_size=(7, 7), stride=3, dilation=(2, 2)),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            ResBlock(dims=64, kernelSize=(3, 3)),
            nn.Conv2d(64, 128, kernel_size=(3, 3), padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            ResBlock(dims=128, kernelSize=(3, 3)),
            nn.Conv2d(128, 256, kernel_size=(3, 3), padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(256),
            ResBlock(dims=256, kernelSize=(3, 3)),
            nn.Conv2d(256, 512, kernel_size=(3, 3), padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(512),
            ResBlock(dims=512, kernelSize=(3, 3)),
        )

        self.fcNet = nn.Linear(512 * 23 * 23, self.outKeypoints * 2)

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
        # convolutional part
        img = self.convPart(img)
        # fc part
        img = img.view(img.size(0), -1)
        img = self.fcNet(img)

        return sigmoid(img)


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
            padding = (int((kernelSize[0] - 1) / 2), int((kernelSize[1] - 1) / 2))
        else:
            padding = int((kernelSize - 1) / 2)

        self.model = nn.Sequential(
            nn.Conv2d(in_channels=dims, out_channels=dims, kernel_size=kernelSize, padding=padding),
            nn.ReLU(),
            # nn.BatchNorm2d(dims),
            nn.Conv2d(in_channels=dims, out_channels=dims, kernel_size=kernelSize, padding=padding),
            nn.ReLU(),
            # nn.BatchNorm2d(dims),
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
        out = self.model(featureVector)
        return out + featureVector
