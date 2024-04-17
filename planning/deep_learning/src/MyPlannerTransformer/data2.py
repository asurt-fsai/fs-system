import torch
import os
from torch.utils.data import DataLoader

class DataSet2():
    
    def __init__(self, srcSeqLength, tgtSeqLength, srcNumofFeatures, tgtNumofFeatures):
        
        self.srcSeqLength = srcSeqLength
        self.tgtSeqLength = tgtSeqLength
        self.srcNumofFeatures = srcNumofFeatures
        self.tgtNumofFeatures = tgtNumofFeatures
        
    def prepareData(self, srcPath, tgtPath, train = True):
        src = torch.empty(0, self.srcNumofFeatures).cuda()
        tgt = torch.empty(0, self.tgtNumofFeatures).cuda()
        if train:
            for files in os.listdir(srcPath+"train"):
                src = torch.cat((src, torch.load(f'{srcPath+"train"}/{files}').cuda())).cuda()
            numberOfSamples = src.size(0)
            for files in os.listdir(tgtPath+"train"):
                tgt = torch.cat((tgt, torch.load(f'{tgtPath+"train"}/{files}').cuda())).cuda()
                if tgt.size(0) > numberOfSamples:
                    break
        else:
            for files in os.listdir(srcPath+"valid"):
                src = torch.cat((src, torch.load(f'{srcPath+"valid"}/{files}').cuda())).cuda()
            numberOfSamples = src.size(0)
            for files in os.listdir(tgtPath+"valid"):
                tgt = torch.cat((tgt, torch.load(f'{tgtPath+"valid"}/{files}').cuda())).cuda()
                if tgt.size(0) > numberOfSamples:
                    break
                  
        srcNumOfSequencesTrain = src.size(0) // self.srcSeqLength
        tgtNumOfSequencesTrain = tgt.size(0) // self.tgtSeqLength
        
        # reshape the tensors
        src = src[:srcNumOfSequencesTrain * self.srcSeqLength, :].view(srcNumOfSequencesTrain, self.srcSeqLength, -1).cuda()
        tgt = tgt[:tgtNumOfSequencesTrain * self.tgtSeqLength, :].view(tgtNumOfSequencesTrain, self.tgtSeqLength, -1).cuda()
    
        return src, tgt