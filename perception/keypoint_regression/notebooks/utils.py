"""Utils used for keypoint regression."""
# mypy: ignore-errors

from datetime import datetime

import torch

import numpy as np

import matplotlib.pyplot as plt


def showImageTensor(img: torch.Tensor, show=True) -> None:
    """
    shows input image

    Parameters:
    -----------
        img: torch.tensor
            image to show
    """
    img = img.cpu().detach().permute(1, 2, 0).squeeze(2).numpy().astype(np.float32).copy()
    plt.imshow(img)
    if show:
        plt.show()


def getFormattedModelName(name: str) -> str:
    """
    gets the datetime in string format

    Parameters:
    -----------
        name: str
            name of the model
    Returns:
    --------
        str:
            formatted datetime in string format
    """
    return name + "_" + str(datetime.utcnow()).split(".", maxsplit=1)[0].replace(":", "-") + ".pth"
