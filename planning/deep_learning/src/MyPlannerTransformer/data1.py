import torch

class DataSet1():
    
    def __init__(self,) -> None:
        pass

    
    def __loadData(self,dir: str) -> torch.tensor: 
        data = torch.load(dir).cuda()
        return data
    
    def prepareData(self,trainDir: str, validDir) -> torch.tensor:
        srcTrain = self.__loadData(f'{trainDir}/src_train.pt')
        tgtTrain = self.__loadData(f'{trainDir}/tgt_train.pt')
        srcValid = self.__loadData(f'{validDir}/src_valid.pt')
        tgtValid = self.__loadData(f'{validDir}/tgt_valid.pt')
        
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
    
