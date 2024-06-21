"""imports to prepare the data for training"""

from typing import List
import os
import torch


class DataSet2:
    """
    A class representing a dataset for training or validation.

    Args:
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

    def prepareData(
        self, srcPath: str, tgtPath: str, train: bool = True
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Prepare the data for training or validation.

        Args:
            srcPath (str): The path to the source data directory.
            tgtPath (str): The path to the target data directory.
            train (bool, default is true): Whether to prepare the
            data for training or validation. Defaults to True.

        Returns:
            tuple[torch.Tensor, torch.Tensor]: A tuple containing
             the prepared source and target data tensors.
        """
        src = torch.empty(0, self.srcNumofFeatures).cuda()
        tgt = torch.empty(0, self.tgtNumofFeatures).cuda()
        if train:
            for files in os.listdir(srcPath + "train"):
                src = torch.cat((src, torch.load(f'{srcPath+"train"}/{files}').cuda())).cuda()
            numberOfSamples = src.size(0)
            for files in os.listdir(tgtPath + "train"):
                tgt = torch.cat((tgt, torch.load(f'{tgtPath+"train"}/{files}').cuda())).cuda()
                if tgt.size(0) > numberOfSamples:
                    break
        else:
            for files in os.listdir(srcPath + "valid"):
                src = torch.cat((src, torch.load(f'{srcPath+"valid"}/{files}').cuda())).cuda()
            numberOfSamples = src.size(0)
            for files in os.listdir(tgtPath + "valid"):
                tgt = torch.cat((tgt, torch.load(f'{tgtPath+"valid"}/{files}').cuda())).cuda()
                if tgt.size(0) > numberOfSamples:
                    break

        srcNumOfSequencesTrain = src.size(0) // self.srcSeqLength
        tgtNumOfSequencesTrain = tgt.size(0) // self.tgtSeqLength

        # reshape the tensors
        src = (
            src[: srcNumOfSequencesTrain * self.srcSeqLength, :]
            .view(srcNumOfSequencesTrain, self.srcSeqLength, -1)
            .cuda()
        )
        tgt = (
            tgt[: tgtNumOfSequencesTrain * self.tgtSeqLength, :]
            .view(tgtNumOfSequencesTrain, self.tgtSeqLength, -1)
            .cuda()
        )

        return src, tgt

    def listFiles(self, directory: str) -> List[str]:
        """
        Helper function to list files in a directory.

        Args:
            directory (str): Directory path.

        Returns:
            List[str]: List of file paths.
        """
        files = []
        for filename in os.listdir(directory):
            if os.path.isfile(os.path.join(directory, filename)):
                files.append(os.path.join(directory, filename))
        return files

    def loadData(
        self, srcFiles: List[str], tgtFiles: List[str]
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Helper function to load data from file paths.

        Args:
            src_files (List[str]): List of paths to source data files.
            tgt_files (List[str]): List of paths to target data files.

        Returns:
            tuple[torch.Tensor, torch.Tensor]: A tuple containing
             the loaded source and target data tensors.
        """
        src = torch.empty(0, self.srcNumofFeatures)
        tgt = torch.empty(0, self.tgtNumofFeatures)

        for srcFile, tgtFile in zip(srcFiles, tgtFiles):
            src = torch.cat((src, torch.load(srcFile).cuda()))
            tgt = torch.cat((tgt, torch.load(tgtFile).cuda()))

        return src, tgt
