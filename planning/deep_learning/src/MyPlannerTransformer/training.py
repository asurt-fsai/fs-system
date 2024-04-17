from myPlannerTransformer import MyPlannerTransformer
from data2 import DataSet2
import torch
from torch.utils.data import DataLoader
import torch.nn as nn
import matplotlib.pyplot as plt
# from data1 import DataSet1

def plotLoss(loss: list, type: str) -> None:
        
        """
        plotLoss is a function that plots the loss of the model

        Args:
            loss (list): it takes a list of loss
            type (str): it takes a string of the type of loss
        """        
        
        plt.plot(loss)
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.title(f'{type} Loss')
        plt.show(block = False)

def trainModel(model, trainLoader, validLoader,criterion, optimizer,epochs = 10):
        
        # Training loop
        VAL_LOSS = []
        TRAIN_LOSS = []

        for epoch in range(epochs):
            model.train()
            total_loss = 0.0

            for batch_idx, (src, tgt) in enumerate(trainLoader):
               

                optimizer.zero_grad()
                output = model(src, tgt[:, :-1, :])  # Use teacher forcing for training

                # Flatten the output and target for computing the loss
                loss = criterion(output.view(-1, 2), tgt[:, 1:, :].contiguous().view(-1, 2))
                loss.backward()
                optimizer.step()

                total_loss += loss.item()

            average_loss = total_loss / len(trainLoader)
            print(f"Epoch [{epoch + 1}/{epochs}], Loss: {average_loss:.4f}")
            TRAIN_LOSS.append(average_loss)

            #Validation Remember to uncomment 
            model.eval()
            with torch.no_grad():
                val_loss = 0.0
                for val_src, val_tgt in validLoader:
                    
                    val_output = model(val_src, val_tgt[:, :-1, :])  # No teacher forcing for validation
                    val_loss += criterion(val_output.view(-1, 2), val_tgt[:, 1:, :].contiguous().view(-1, 2)).item()

            average_val_loss = val_loss / len(validLoader)
            print(f"Validation Loss: {average_val_loss:.4f}")
            VAL_LOSS.append(average_val_loss)
            

        return TRAIN_LOSS, VAL_LOSS
    
model = MyPlannerTransformer()
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
model.to('cuda')

# data1 = DataSet1()
# srcTrain, tgtTrain, srcValid, tgtValid = data1.prepareData('/inputs/tensors/train', '/inputs/tensors/valid')

# trainData = list(zip(srcTrain, tgtTrain))
# validData = list(zip(srcValid, tgtValid))

# trainLoader = DataLoader(trainData, batch_size=100, shuffle=True)
# validLoader = DataLoader(validData, batch_size=100, shuffle=True)

data = DataSet2()
src_path = "inputs/tensorsV2/inputs/"  # Replace with the path to your directory
tgt_path = "inputs/tensorsV2/outputs/"

srcTrain, tgtTrain, srcValid, tgtValid = data.prepareData(src_path, tgt_path)

trainData = list(zip(srcTrain, tgtTrain))
validData = list(zip(srcValid, tgtValid))

trainLoader = DataLoader(trainData, batch_size=100, shuffle=True)
validLoader = DataLoader(validData, batch_size=100, shuffle=True)

trainLoss, valLoss = trainModel(model, trainLoader, validLoader, criterion, optimizer, epochs=3)

# save model
torch.save(model.state_dict(), 'src/MyPlannerTransformer/.pt/myPlannerTransformer.pt')

plotLoss(trainLoss, "Train")
plt.pause(3)
plt.close()
plotLoss(valLoss, "Validation")
plt.pause(3)
plt.close()

