import torch
import os
from torch.utils.data import DataLoader

class DataSet2():
    
    def __init__(self) -> None:
        pass
    
    def prepareData(self, srcPath, tgtPath):
        srcTrain = torch.empty(0, 5).cuda()
        tgtTrain = torch.empty(0, 2).cuda()
        srcValid = torch.empty(0, 5).cuda()
        tgtValid = torch.empty(0, 2).cuda()
        for files in os.listdir(srcPath+"train"):
            srcTrain = torch.cat((srcTrain, torch.load(f'{srcPath+"train"}/{files}').cuda())).cuda()

        for files in os.listdir(tgtPath+"train"):
            tgtTrain = torch.cat((tgtTrain, torch.load(f'{tgtPath+"train"}/{files}').cuda())).cuda()
        
        for files in os.listdir(srcPath+"valid"):
            srcValid = torch.cat((srcValid, torch.load(f'{srcPath+"valid"}/{files}').cuda())).cuda()
        
        for files in os.listdir(tgtPath+"valid"):
            tgtValid = torch.cat((tgtValid, torch.load(f'{tgtPath+"valid"}/{files}').cuda())).cuda()
                  
        srcSeqLength = 10
        srcNumOfSequencesTrain = srcTrain.size(0) // srcSeqLength
        tgtSeqLength = 4
        tgtNumOfSequencesTrain = tgtTrain.size(0) // tgtSeqLength
        srcNumOfSequencesValid = srcValid.size(0) // srcSeqLength
        tgtNumOfSequencesValid = tgtValid.size(0) // tgtSeqLength
        # reshape the tensors
        srcTrain = srcTrain[:srcNumOfSequencesTrain * srcSeqLength, :].view(srcNumOfSequencesTrain, srcSeqLength, -1).cuda()
        tgtTrain = tgtTrain[:tgtNumOfSequencesTrain * tgtSeqLength, :].view(tgtNumOfSequencesTrain, tgtSeqLength, -1).cuda()
        srcValid = srcValid[:srcNumOfSequencesValid * srcSeqLength, :].view(srcNumOfSequencesValid, srcSeqLength, -1).cuda()
        tgtValid = tgtValid[:tgtNumOfSequencesValid * tgtSeqLength, :].view(tgtNumOfSequencesValid, tgtSeqLength, -1).cuda()
    
        return srcTrain, tgtTrain, srcValid, tgtValid
    
