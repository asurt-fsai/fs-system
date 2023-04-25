"""postprocessing used to postprocess the output of the neural network."""
# mypy: ignore-errors
import numpy.typing as npt


class PostProcessor:
    """
    post processor class used to postprocess the output of the neural network.
    """

    def __call__(
        self,
        keyPointArray: npt.NDArray,
        imageSize: npt.NDArray,
        topLeftBbox: npt.NDArray,
    ) -> npt.NDArray:
        """
        calls the postProcessingStep function


        """
        self.postProcessingStep(keyPointArray, imageSize, topLeftBbox)

    def postProcessingStep(
        self,
        keyPointArray: npt.NDArray,
        imageSize: npt.NDArray,
        topLeftBbox: npt.NDArray,
    ) -> npt.NDArray:
        """
        This function is used to postprocess the output of the neural network.

            Parameters:
            ----------
            keyPointArray: numpy.ndarray
                The output of the neural network.
            imageSize: numpy.ndarray
                The size of the image.
            topLeftBbox: numpy.ndarray
                The top left corner of the image.

            Returns:
            --------
            numpy.ndarray:
                The postprocessed output of the neural network.
        """
        keyPointArray = keyPointArray.view(-1, 2)
        return (keyPointArray * imageSize + topLeftBbox).numpy()
