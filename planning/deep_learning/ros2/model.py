"""imports for the model"""
from typing import Optional, Any
import torch
from torch import nn


SEED = 42
torch.manual_seed(SEED)
# torch.cuda.manual_seed(seed)


class MyPlannerTransformer(nn.Module):
    """
    MyPlannerTransformer is a class that represents a transformer-based planner model.

    Args:
        d_model (int): The dimension of the model (default: 16).
        nhead (int): The number of attention heads (default: 2).
        num_encoder_layers (int): The number of encoder layers (default: 2).
        num_decoder_layers (int): The number of decoder layers (default: 2).
        dropout (float): The dropout rate (default: 0.1).
        dim_feedforward (int): The dimension of the feedforward network (default: 2048).

    Attributes:
        transformer (nn.Transformer): The transformer module.
        input (nn.Linear): Linear layer to map the source tensor to the model's dimension.
        tgt (nn.Linear): Linear layer to map the target tensor to the model's dimension.
        output (nn.Linear): Linear layer to map the model's output to the target tensor's dimension.
    """

    def __init__(
        self,
        d_model: int = 16,
        nhead: int = 2,
        num_encoder_layers: int = 2,
        num_decoder_layers: int = 2,
        dropout: float = 0.1,
        dim_feedforward: int = 2048,
    ) -> None:
        super().__init__()
        self.transformer = nn.Transformer(
            d_model,
            nhead,
            num_encoder_layers,
            num_decoder_layers,
            dim_feedforward,
            dropout,
            batch_first=True,
        )
        # map tgt to have the same number of features as src
        self.input = nn.Linear(6, d_model)
        self.tgt = nn.Linear(2, d_model)
        self.output = nn.Linear(d_model, 2)

    def forward(
        self,
        src: torch.Tensor,
        tgt: torch.Tensor,
    ) -> Any:
        """
        Perform forward pass of the model.

        Args:
            src (torch.tensor): The source tensor.
            tgt (torch.tensor): The target tensor.

        Returns:
            torch.tensor: The output tensor.
        """
        src = self.input(src)
        tgt = self.tgt(tgt)
        out = self.transformer(src, tgt)
        out = out[:, :, :2]
        return out

    def predict(self, src: Any, tgt: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        predict is a function that uses the transformer model to predict the next waypoints
        it works by auto-regressive property of the transformer model
        this function is used to predict the entire sequence at once

        Args:
            src (torch.tensor):

            takes a tensor of shape (sequence, 6)
            in our case sequence is 15 and 6 is the number of features

        Returns:
            torch.tensor:

            tgt tensor of shape (outputSequence, 2)
            outputSequence is the number of waypoints to predict
            2 is the number of features
        """

        self.eval()
        src = src.reshape(1, -1, 6)
        if tgt is None:
            tgt = torch.zeros((1, 1, 2))
        else:
            tgt = tgt.reshape(1, -1, 2)
        with torch.no_grad():
            for _ in range(15):
                prediction = self.forward(src, tgt)
                tgt = torch.cat((tgt, prediction[:, -1:]), dim=1)

        return tgt.reshape(-1, 2)


def createModel(path: str) -> MyPlannerTransformer:
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
    model.load_state_dict(torch.load(path, map_location=torch.device("cpu")))

    # model.cuda()
    return model
