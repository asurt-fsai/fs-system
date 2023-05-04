# mypy: ignore-errors
"""trainer module
"""

from dataclasses import dataclass, field
from typing import List

import torch

from torch.utils.data import DataLoader
from torch.optim import Optimizer


@dataclass
class TrainingParameters:  # pylint: disable=too-many-instance-attributes
    """
    training parameters class
    used by trainer to train the model
    """

    model: torch.nn.Module
    criterion: torch.nn.Module
    optimizer: Optimizer
    trainloader: DataLoader
    validloader: DataLoader
    epochs: int
    savedModelName: str
    printEvery: int = field(default_factory=lambda: 2)
    scheduler: torch.optim.lr_scheduler = field(default_factory=lambda: None)
    trainingTracker: List[int] = field(default_factory=list)
    validTracker: List[int] = field(default_factory=list)


def train(trainingParameters: TrainingParameters):  # pylint: disable=too-many-locals
    """trains the model using the given parameters

    Parameters
    ----------
    trainingParameters : TrainingParameters
        training parameters used to train the model
    """
    # unpack the training parameters
    model = trainingParameters.model
    criterion = trainingParameters.criterion
    optimizer = trainingParameters.optimizer
    trainloader = trainingParameters.trainloader
    validloader = trainingParameters.validloader
    epochs = trainingParameters.epochs
    printEvery = trainingParameters.printEvery
    savedModelName = trainingParameters.savedModelName
    scheduler = trainingParameters.scheduler
    trainingTracker = trainingParameters.trainingTracker
    validTracker = trainingParameters.validTracker

    # get current device
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = model.to(device)
    print("Training on", device)
    # track the best valid loss
    bestValidLoss = float("inf")

    # train the model
    for epoch in range(1, epochs + 1):
        trainloss = 0
        validloss = 0

        # train the model
        model.train()
        for each in trainloader:
            # extract and add to device
            imgs, keypoints = each
            batchSize, *_ = imgs.shape
            imgs, keypoints = imgs.to(device), keypoints.to(device)
            # forward pass
            out = model(imgs)
            loss = criterion(keypoints, out)
            # backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # update the training tracker
            trainloss += loss.item() * batchSize

        # validate the model
        model.eval()
        with torch.no_grad():
            for each in validloader:
                imgs, keypoints = each
                batchSize, *_ = imgs.shape
                imgs, keypoints = imgs.to(device), keypoints.to(device)
                out = model(imgs)
                loss = criterion(keypoints, out)

                validloss += loss.item() * batchSize

        trainloss /= len(trainloader.dataset)
        validloss /= len(validloader.dataset)

        trainingTracker.append(trainloss)
        validTracker.append(validloss)

        # prompt model progress
        if epoch % printEvery == 0:
            print(
                f"Epoch {epoch}/{epochs}.. "
                f"Train loss: {trainloss:.5f}.. "
                f"Valid loss: {validloss:.5f}.. "
            )
        # save the model
        if validloss < bestValidLoss:
            print(
                f"Validation loss decreased ({bestValidLoss:.6f}"
                f" --> {validloss:.6f}).  Saving model ..."
            )
            bestValidLoss = validloss
            torch.save(model.state_dict(), savedModelName)

        # update the learning rate
        if scheduler:
            scheduler.step(validloss)
