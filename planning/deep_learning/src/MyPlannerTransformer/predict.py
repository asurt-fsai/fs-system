import torch
from typing import Optional
from myPlannerTransformer import MyPlannerTransformer

# load model

def createModel():
    
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
    model.to('cuda')
    model.load_state_dict(torch.load('src/myPlannerTransformer/.pt/myPlannerTransformer.pt'))
    return model


def pathPredict(model, src: torch.tensor,tgt: Optional[torch.tensor] = None):
        
    try:
        last_prediction = torch.load('src/myPlannerTransformer/.pt/lastWaypoint.pt')
        output = model.predict(src, last_prediction)
        torch.save(output[-1].reshape(1,-1,2), 'src/myPlannerTransformer/.pt/lastWaypoint.pt')
        return output
    except:
        output = model.predict(src)
        torch.save(output[-1].reshape(1,-1,2), 'src/myPlannerTransformer/.pt/lastWaypoint.pt')
        return output

