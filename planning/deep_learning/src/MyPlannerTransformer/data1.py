"""importing libraries for the data set"""

from typing import Any
import torch


class DataSet1:
    """
    A class representing a dataset.

    Attributes:
        srcSeqLength (int): The length of the source sequence.
        tgtSeqLength (int): The length of the target sequence.
        srcNumofFeatures (int): The number of features in the source data.
        tgtNumofFeatures (int): The number of features in the target data.
    """

    def __init__(
        self, srcSeqLength: int, tgtSeqLength: int, srcNumofFeatures: int, tgtNumofFeatures: int
    ) -> None:
        self.srcSeqLength = srcSeqLength
        self.tgtSeqLength = tgtSeqLength
        self.srcNumofFeatures = srcNumofFeatures
        self.tgtNumofFeatures = tgtNumofFeatures

    def loadData(self, direc: str) -> Any:
        """
        Loads data from the specified directory.

        Args:
            dir (str): The directory path where the data is stored.

        Returns:
            Any: The loaded data.
        """
        data = torch.load(direc).cuda()
        return data

    def prepareData(self, trainDir: str, validDir: str) -> Any:
        """
        Prepare the data for training and validation.

        Args:
            trainDir (str): The directory path for the training data.
            validDir (str): The directory path for the validation data.

        Returns:
            torch.tensor: A tuple containing the prepared training and validation data.
                - srcTrain: The prepared source training data.
                - tgtTrain: The prepared target training data.
                - srcValid: The prepared source validation data.
                - tgtValid: The prepared target validation data.
        """
        srcTrain = self.loadData(f"{trainDir}/src_train.pt")
        tgtTrain = self.loadData(f"{trainDir}/tgt_train.pt")
        srcValid = self.loadData(f"{validDir}/src_valid.pt")
        tgtValid = self.loadData(f"{validDir}/tgt_valid.pt")

        srcSeqLength = 10
        srcNumOfSequencesTrain = srcTrain.size(0) // srcSeqLength
        tgtSeqLength = 4
        tgtNumOfSequencesTrain = tgtTrain.size(0) // tgtSeqLength
        srcNumOfSequencesValid = srcValid.size(0) // srcSeqLength
        tgtNumOfSequencesValid = tgtValid.size(0) // tgtSeqLength
        # reshape the tensors
        srcTrain = (
            srcTrain[: srcNumOfSequencesTrain * srcSeqLength, :]
            .view(srcNumOfSequencesTrain, srcSeqLength, -1)
            .cuda()
        )
        tgtTrain = (
            tgtTrain[: tgtNumOfSequencesTrain * tgtSeqLength, :]
            .view(tgtNumOfSequencesTrain, tgtSeqLength, -1)
            .cuda()
        )
        srcValid = (
            srcValid[: srcNumOfSequencesValid * srcSeqLength, :]
            .view(srcNumOfSequencesValid, srcSeqLength, -1)
            .cuda()
        )
        tgtValid = (
            tgtValid[: tgtNumOfSequencesValid * tgtSeqLength, :]
            .view(tgtNumOfSequencesValid, tgtSeqLength, -1)
            .cuda()
        )

        return srcTrain, tgtTrain, srcValid, tgtValid
