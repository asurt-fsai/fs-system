import torch
import torch.nn as nn
from typing import Optional

class MyPlannerTransformer(nn.Module):
    """_summary_

    Args:
        nn (_type_): _description_
    """    
    def __init__(self, num_features=6, numOfEncoderLayers = 6, numOfDecoderLayers=6) -> None:
        super().__init__()
        self.transformer_model = nn.Transformer(
            d_model=num_features,
            nhead=3,
            num_encoder_layers=numOfEncoderLayers,
            num_decoder_layers=numOfDecoderLayers,
            dim_feedforward=2048,
            dropout=0.1,
        )
        # map tgt to have the same number of features as src
        self.tgt = nn.Linear(2, num_features)

    def forward(
        self,
        src: torch.tensor,
        tgt: torch.tensor,
        src_mask: Optional[torch.tensor]=None,
        tgt_mask: Optional[torch.tensor]=None,
        memory_mask: Optional[torch.tensor]=None,
        src_key_padding_mask: Optional[torch.tensor]=None,
        tgt_key_padding_mask: Optional[torch.tensor]=None,
        memory_key_padding_mask: Optional[torch.tensor]=None,
        max_len: Optional[int]=None,
    ) -> torch.tensor:
        """
        Performs the forward pass of the transformer model.

        Args:
            src (torch.tensor): The source tensor with size (number of cones x number of features).
            tgt (torch.tensor): The target tensor with size (number of waypoints x number of features).
            src_mask (Optional[torch.tensor], optional): The mask applied to the source tensor. Defaults to None.
            tgt_mask (Optional[torch.tensor], optional): The mask applied to the target tensor. Defaults to None.
            memory_mask (Optional[torch.tensor], optional): The mask applied to the memory tensor. Defaults to None.
            src_key_padding_mask (Optional[torch.tensor], optional): The padding mask applied to the source tensor. Defaults to None.
            tgt_key_padding_mask (Optional[torch.tensor], optional): The padding mask applied to the target tensor. Defaults to None.
            memory_key_padding_mask (Optional[torch.tensor], optional): The padding mask applied to the memory tensor. Defaults to None.
            max_len (Optional[int], optional): The maximum length of the output tensor. Defaults to None.

        Returns:
            torch.tensor: The output of the transformer model with the same size as the target tensor.
        """
        tgt = self.tgt(tgt)
        out = self.transformer_model(src, tgt)
        return out[:, :2]
    
        
    
    def predict(self, src: torch.tensor, tgt: torch.tensor, max_len: Optional[int]=None)->torch.tensor:
        
        """
        Predicts the output based on the given source and target tensors.

        Args:
            src (torch.tensor): The source tensor.
            tgt (torch.tensor): The target tensor.
            max_len (Optional[int], optional): The maximum length of the prediction. Defaults to None.

        Returns:
            torch.tensor: The predicted output tensor.
        """  
        self.eval()
        with torch.no_grad():
            prediction = self.forward(src, tgt)
        return prediction
    
    def save_model(self, path: str)->None:
        """
        Saves the model to the given path.

        Args:
            path (str): The path to save the model.
        """        
        torch.save(self.state_dict(), path)
    
    def load_model(self, path: str)->None:
        """
        Loads the model from the given path.

        Args:
            path (str): The path to load the model.
        """        
        self.load_state_dict(torch.load(path))
        self.eval()
    
    