"""these imports are used to load the model and predict the path"""
from typing import Any
import torch
from myPlannerTransformer import MyPlannerTransformer
from torch import Tensor


def createModel() -> MyPlannerTransformer:
    """
    Create a planning model from a saved state dict

    Parameters
    ----------
    path : str
        Path to the saved state dict

    Returns
    -------
    MyPlannerTransformer
        Planning model with the saved state dict loaded
    """
    model = MyPlannerTransformer()
    model.to("cuda")
    model.load_state_dict(torch.load("src/myPlannerTransformer/.pt/myPlannerTransformer.pt"))
    return model


def pathPredict(model: MyPlannerTransformer, src: Any) -> Tensor:
    """
    Predicts the path using the given model and source tensor.

    Args:
        model (MyPlannerTransformer): The model used for prediction.
        src (torch.Tensor): The source tensor for prediction.

    Returns:
        torch.Tensor: The predicted path tensor.
    """
    try:
        lastPrediction = torch.load("../notebooks/lastPredictions.pt")
        output = model.predict(src, lastPrediction)
        torch.save(
            torch.index_select(output, 0, torch.tensor([-1])).reshape(1, -1, 2),
            "../notebooks/lastPredictions.pt",
        )
        return output
    except FileNotFoundError:
        output = model.predict(src)
        torch.save(
            torch.index_select(output, 0, torch.tensor([-1])).reshape(1, -1, 2),
            "../notebooks/lastPredictions.pt",
        )
        return output
