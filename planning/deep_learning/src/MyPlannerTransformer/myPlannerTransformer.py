import torch
import torch.nn as nn
from typing import Optional
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
import os

seed = 42
torch.manual_seed(seed)
torch.cuda.manual_seed(seed)

class MyPlannerTransformer(nn.Module):
    
    def __init__(self, d_model=16,nhead = 2,num_encoder_layers = 2, num_decoder_layers=2, dropout = 0.1, dim_feedforward = 2048) -> None:
        super().__init__()
        self.transformer = nn.Transformer(
            d_model,
            nhead,
            num_encoder_layers,
            num_decoder_layers,
            dim_feedforward,
            dropout,
            batch_first=True
        )
        # map tgt to have the same number of features as src
        self.input = nn.Linear(5, d_model)
        self.tgt = nn.Linear(2, d_model)
        self.output = nn.Linear(d_model, 2)

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
        
    )->torch.tensor:
        src = self.input(src)
        tgt = self.tgt(tgt)
        out = self.transformer(src, tgt)
        out = out[:,:,:2]
        return out
    
    
    def predict(self, src: torch.tensor, tgt: Optional[torch.tensor] = None) -> torch.tensor:
        
        """
        predict v3 is a function that uses the transformer model to predict the next waypoints
        it works by auto-regressive property of the transformer model
        this function is used to predict the entire sequence at once
        
        Args:
            src (torch.tensor):
            
            takes a tensor of shape (sequence, 5)
            in our case sequence is 10 and 5 is the number of features
            
        Returns:
            torch.tensor:
            
            tgt tensor of shape (outputSequence, 2)
            outputSequence is the number of waypoints to predict
            2 is the number of features
        """
        
        self.eval()
        src = src.reshape(-1, 10, 5)
        if tgt is None:
            tgt = torch.zeros((1, 1, 2)).cuda()
        with torch.no_grad():
                for _ in range(4):
                    prediction = self.forward(src, tgt)
                    tgt = torch.cat((tgt, prediction[:,-1:]), dim=1)

        return tgt.reshape(-1,2)