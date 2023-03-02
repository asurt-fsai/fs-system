"""
This file is used to postprocess the output of the neural network.
"""
import datetime
import numpy as np
import numpy.typing as npt


def postProcessing(
    keyPointArray: npt.NDArray[np.float32],
    imageSize: npt.NDArray[np.float32],
    topLeftBbox: npt.NDArray[np.float32],
) -> npt.NDArray[np.float32]:
    """
    This function is used to postprocess the output of the neural network.

        Parameters:
        keyPointArray (numpy.ndarray): The output of the neural network.
        imageSize (numpy.ndarray): The size of the image.
        topLeftBbox (numpy.ndarray): The top left corner of the image.

        Returns:
        numpy.ndarray: The postprocessed output of the neural network.
    """
    postArray = (
        np.hstack(
            (
                imageSize,
                imageSize,
                imageSize,
                imageSize,
                imageSize,
                imageSize,
                imageSize,
                imageSize,
            )
        )
        * keyPointArray
    )
    postArray += np.hstack(
        (
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
            topLeftBbox,
        )
    )
    return postArray


if __name__ == "__main__":
    keyPointArray1 = np.random.randint(0, 54, (610, 16))
    for_resizing = np.random.randint(0, 54, (610, 2))
    topLeftBbox1 = np.random.randint(0, 54, (610, 2))
    SUM: float = 0
    ITERS = 100000

    for i in range(ITERS):
        a = datetime.datetime.now()
        postProcessing(keyPointArray1, for_resizing, topLeftBbox1)
        b = datetime.datetime.now()
        SUM += (b - a).microseconds * 1e-3

    print(SUM / ITERS, "milliseconds")
