"""importing the necessary libraries for training the model"""

from typing import Any, List, Tuple
from myPlannerTransformer import MyPlannerTransformer
from data2 import DataSet2
import torch
from torch.utils.data import DataLoader
from torch.utils.data import Dataset
from torch import nn
import matplotlib.pyplot as plt


def plotLoss(loss: list[float], lossType: str) -> None:
    """
    plotLoss is a function that plots the loss of the model

    Args:
        loss (list[float]): it takes a list of loss
        lossType (str): it takes a string of the type of loss
    """

    plt.plot(loss)
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.title(f"{lossType} Loss")


def trainOneEpoch(
    model2: nn.Module,
    trainLoader2: DataLoader[Any],
    criterion2: nn.Module,
    optimizer2: torch.optim.Optimizer,
) -> float:
    """
    Trains the model for one epoch using the given data loader and optimizer.

    Args:
        model2 (nn.Module): The model to be trained.
        trainLoader2 (DataLoader[Any]): The data loader containing the training data.
        criterion2 (nn.Module): The loss function used for training.
        optimizer2 (torch.optim.Optimizer): The optimizer used for updating the model's parameters.

    Returns:
        float: The average loss for the epoch.
    """
    model2.train()
    totalLoss = 0.0
    for src, tgt in trainLoader2:
        optimizer2.zero_grad()
        output = model2(src, tgt[:, :-1, :])
        loss = criterion2(output.view(-1, 2), tgt[:, 1:, :].contiguous().view(-1, 2))
        loss.backward()
        optimizer2.step()
        totalLoss += loss.item()
    averageLoss = totalLoss / len(trainLoader2)
    return averageLoss


def validateOneEpoch(
    model3: nn.Module,
    validLoader3: DataLoader[Any],
    criterion3: nn.Module,
) -> float:
    """
    Validates the model for one epoch using the given validation data loader and criterion.

    Args:
        model3 (nn.Module): The model to be validated.
        validLoader3 (DataLoader[Any]): The validation data loader.
        criterion3 (nn.Module): The loss criterion.

    Returns:
        float: The average validation loss.
    """
    model3.eval()
    valLoss3 = 0.0
    with torch.no_grad():
        for valSrc, valTgt in validLoader3:
            valOutput = model3(valSrc, valTgt[:, :-1, :])
            valLoss3 += criterion3(
                valOutput.view(-1, 2), valTgt[:, 1:, :].contiguous().view(-1, 2)
            ).item()
    averageValLoss = valLoss3 / len(validLoader3)
    return averageValLoss


def trainModel(
    model1: nn.Module,
    trainLoader1: DataLoader[Any],
    validLoader1: DataLoader[Any],
    criterion1: nn.Module,
    optimizer1: torch.optim.Optimizer,
    epochs: int = 10,
) -> Tuple[List[float], List[float]]:
    """
    Trains the given model using the provided data
    loaders and criterion for a specified number of epochs.

    Args:
        model (nn.Module): The model to be trained.
        trainLoader (DataLoader[Any]): The data loader for the training data.
        validLoader1 (DataLoader[Any]): The data loader for the validation data.
        criterion1 (nn.Module): The loss criterion for training.
        optimize1r (torch.optim.Optimizer): The optimizer for training.
        epochs (int, optional): The number of epochs to train the model (default: 10).

    Returns:
        Tuple[List[float], List[float]]: A tuple containing two
        lists of floats representing the training and validation losses for each epoch.
    """
    trainLosses: List[float] = []
    valLosses: List[float] = []

    for epoch in range(epochs):
        trainLoss1 = trainOneEpoch(model1, trainLoader1, criterion1, optimizer1)
        trainLosses.append(trainLoss1)
        print(f"Epoch [{epoch + 1}/{epochs}], Loss: {trainLoss:.4f}")

        valLoss1 = validateOneEpoch(model1, validLoader1, criterion1)
        valLosses.append(valLoss1)
        print(f"Validation Loss: {valLoss1:.4f}")

    return trainLosses, valLosses


model = MyPlannerTransformer()
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
model.to("cuda")

data = DataSet2(srcSeqLength=10, tgtSeqLength=10, srcNumofFeatures=5, tgtNumofFeatures=2)
SRCPATH = "inputs/tensorsV2/inputs/"  # Replace with the path to your directory
TGTPATH = "inputs/tensorsV2/outputs/"


class MyDataset(Dataset[Tuple[Any, Any]]):
    """
    A custom dataset class for handling data in the MyPlannerTransformer model.

    Args:
        dataS (List[Tuple[Any, Any]]): The input data as a list of tuples.

    Attributes:
        dataS (List[Tuple[Any, Any]]): The input data as a list of tuples.

    Methods:
        __len__(): Returns the length of the dataset.
        __getitem__(index: int): Returns the item at the specified index.

    """

    def __init__(self, dataS: List[Tuple[torch.Tensor, torch.Tensor]]):
        self.dataS = dataS

    def __len__(self) -> int:
        return len(self.dataS)

    def __getitem__(self, index: int) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.dataS[index]


srcTrain, tgtTrain = data.prepareData(SRCPATH, TGTPATH, train=True)
srcValid, tgtValid = data.prepareData(SRCPATH, TGTPATH, train=False)

perm = torch.randperm(srcTrain.size(0))
srcTrain = srcTrain[perm]

trainData: List[Tuple[torch.Tensor, torch.Tensor]] = list(zip(srcTrain, tgtTrain))
validData: List[Tuple[torch.Tensor, torch.Tensor]] = list(zip(srcValid, tgtValid))

trainLoader = DataLoader(MyDataset(trainData), batch_size=100, shuffle=True)
validLoader = DataLoader(MyDataset(validData), batch_size=100, shuffle=True)

trainLoss, valLoss = trainModel(model, trainLoader, validLoader, criterion, optimizer, epochs=3)

# save model
torch.save(model.state_dict(), "src/MyPlannerTransformer/.pt/myPlannerTransformer.pt")
plotLoss(trainLoss, "Train")
plt.pause(3)
plt.close()
plotLoss(valLoss, "Validation")
plt.pause(3)
plt.close()
