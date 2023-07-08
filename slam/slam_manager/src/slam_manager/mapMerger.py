"""
MapMerger class used to merge pointclouds from aloam
"""
import os
import datetime
from typing import List

import numpy as np
import numpy.typing as npt
import open3d as o3d
from tqdm import tqdm


class MapMerger:
    """
    MapMerger class used to merge the pointclouds from aloam into one map

    Parameters
    ----------
    dataDir: str
        Path to the directory where the aloam pointclouds are saved
    """

    def __init__(self, dataDir: str) -> None:
        self.dataDir = dataDir
        self.pcdCombinedForViz = o3d.geometry.PointCloud()
        self.nodeSkip = 1
        self.scanDir = os.path.join(dataDir, "Scans")
        self.scanFiles: List[str] = []

        self.poses: List[npt.NDArray[np.float64]] = []
        self.scanIdxRangeToStack = [0, 0]

    def readPoses(self) -> None:
        """
        Reads the optimized poses from the file optimized_poses.txt
        """
        with open(
            os.path.join(self.dataDir, "optimized_poses.txt"), "r", encoding="utf-8"
        ) as posesFile:
            while True:
                line = posesFile.readline()
                if not line:
                    break
                poseSE3 = np.asarray([float(i) for i in line.split()])
                poseSE3 = np.vstack((np.reshape(poseSE3, (3, 4)), np.asarray([0, 0, 0, 1])))
                self.poses.append(poseSE3)
            self.scanIdxRangeToStack = [0, len(self.poses)]

    def mergeMap(self, saveLatestOnly: bool = True) -> None:
        """
        Merges the aloam maps and saves the merged map

        Parameters
        ----------
        saveLatestOnly: bool
            If true, always rewrites the latest.pcd map,
            otherwise, saves the map with timestamp as its name
        """
        self.readPoses()
        self.scanFiles = os.listdir(self.scanDir)
        self.scanFiles.sort()
        if self.scanIdxRangeToStack[1] < 1:
            return
        assert self.scanIdxRangeToStack[1] > self.scanIdxRangeToStack[0]

        nodesCount = 0
        for nodeIdx in tqdm(range(self.scanIdxRangeToStack[0], self.scanIdxRangeToStack[1])):
            nodesCount += 1
            if nodesCount % self.nodeSkip != 0:
                if nodeIdx != self.scanIdxRangeToStack[0]:  # to ensure the vis init
                    continue

            scanPose = self.poses[nodeIdx]

            scanPath = os.path.join(self.scanDir, self.scanFiles[nodeIdx])
            scanPcd = o3d.io.read_point_cloud(scanPath)

            scanPcdGlobal = scanPcd.transform(
                scanPose
            )  # global coord, note that this is not deepcopy

            self.pcdCombinedForViz += scanPcdGlobal  # open3d pointcloud class append is fast

        savedMapNames = [os.path.join(self.dataDir, "Maps/latest.pcd")]

        if not saveLatestOnly:
            timeNow = datetime.datetime.now()
            timeNowStr = timeNow.strftime("%m_%d_%H_%M_%S")
            filename = (
                str(self.scanIdxRangeToStack[0])
                + "_"
                + str(self.scanIdxRangeToStack[1])
                + "_"
                + timeNowStr
                + ".pcd"
            )
            savedMapNames.append(os.path.join(self.dataDir, "Maps/", filename))
        self.saveMap(savedMapNames)

    def saveMap(self, fileNames: List[str]) -> None:
        """
        Save the current map in the given fileNames

        Parameters
        ----------
        fileNames: List[str]
            A copy of the map is save in each of the give filenames
        """
        for name in fileNames:
            o3d.io.write_point_cloud(name, self.pcdCombinedForViz)
