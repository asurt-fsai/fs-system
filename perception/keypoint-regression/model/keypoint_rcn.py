""" this file provdides the implementation for keypoint regression
"""
# mypy: ignore-errors
# pylint: disable=too-few-public-methods
import torch  # pylint: disable=import-error
from torch import nn  # pylint: disable=import-error


class KeypointRCN(nn.Module):
    """keypoint regression class"""

    def __init__(self, outKeypoints: int = 7):
        """constructor

        Parameters:
        -----------
            outKeypoints (int, optional): _description_. Defaults to 7.
        """
        super().__init__()
        self.outKeypoints = outKeypoints

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        """performs forward propagation

        Parameters:
        -----------
        img: torch.Tensor
            input image for the model to extract
            features

        Returns:
        --------
        torch.Tensor
            shape = (batch_size,7) representing the keypoints of the cone
            in the input image relative to the image frame
        """
        return img


class ResBlock(nn.Module):
    """Represents Residual blocks from resenet paper
    Deep Residual Learning for Image Recognition  https://arxiv.org/abs/1512.03385
    """

    def __init__(self, dims: int = 16, kernelSize: int = 3):
        """constructor

        Parameters:
        -----------
        dims:(int, optional)
            input and output dimension
        kernelSize (int, optional):
            convolution kernel size
        """
        super().__init__()

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
