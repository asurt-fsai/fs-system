"""Utils used for keypoint regression."""

import torch

import matplotlib.pyplot as plt


def showImageTensor(img: torch.Tensor) -> None:
    """
    shows input image

    Parameters:
    -----------
        img: torch.tensor
            image to show
    """
    img = img.detach().squeeze(0).permute((1, 2, 0)).cpu().numpy()
    plt.imshow(img)
    plt.show()
