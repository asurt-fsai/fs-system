"""
Unittests for the Smornn class
"""
from typing import List
import unittest
import numpy.typing as npt
import numpy as np

from smornn import Smornn


class SmornnTest(unittest.TestCase):
    """
    Test cases for the Smornn class
    """

    minDistsNeighbor = 0.5
    noiseVariance = 0.1

    def setUp(self) -> None:
        """
        Set seed for all tests
        """
        np.random.seed(49)

    def testEasy(self) -> None:
        """
        Tests smoreo and lidar having the same position
        """
        nCones = 100
        smornn = Smornn(self.minDistsNeighbor)

        lidarPoints = np.random.normal(0, 20, (nCones, 2))
        colors = np.random.randint(0, 3, (nCones, 1))
        smoreo = np.hstack((lidarPoints, colors, np.random.rand(nCones, 1)))

        smornn.lidarCallback(lidarPoints)
        smornn.smoreoCallback(smoreo)
        out = smornn.run()
        self.assertTrue(np.all(out == smoreo))

    def testMedium(self) -> None:
        """
        Tests smoreo and lidar having slightly different positions
        """
        nCones = 100
        smornn = Smornn(self.minDistsNeighbor)

        lidarPoints = np.random.normal(0, 20, (nCones, 2))
        colors = np.random.randint(0, 3, (nCones, 1))
        noise = np.random.normal(0, self.noiseVariance, (nCones, 2))
        smoreo = np.hstack((lidarPoints + noise, colors, np.random.rand(nCones, 1)))

        smornn.lidarCallback(lidarPoints)
        smornn.smoreoCallback(smoreo)
        out = smornn.run()
        groundTruth = smoreo
        groundTruth[:, :2] = lidarPoints
        self.assertTrue(np.all(out == groundTruth))

    def addExtraCones(
        self, cones: npt.NDArray[np.float64], nExtra: int = 0
    ) -> npt.NDArray[np.float64]:
        """
        Add extra cones to an existing array, making sure they are farther than a certain dist
        """
        extraCones: List[npt.NDArray[np.float64]] = []
        while len(extraCones) < nExtra:
            cone = np.random.normal(0, 20, (1, 2))
            closestDist = np.min(np.linalg.norm(cones - cone, axis=1))
            if closestDist > self.minDistsNeighbor:
                extraCones.append(cone.reshape(-1))
        return np.array(extraCones)

    def testHard(self) -> None:
        """
        Tests smoreo and lidar having different positions with outliers in both
        """
        nCones = 100
        nExtraCones = 10
        smornn = Smornn(self.minDistsNeighbor)

        lidarPoints = np.random.normal(0, 20, (nCones, 2))
        onlyLidarExtraPoints = self.addExtraCones(lidarPoints, nExtraCones)
        allLidarPoints = np.vstack((lidarPoints, onlyLidarExtraPoints))

        onlySmoreoExtraPoints = self.addExtraCones(allLidarPoints, nExtraCones)
        onlySmoreoPoints = np.vstack((lidarPoints, onlySmoreoExtraPoints))

        colors = np.random.randint(0, 3, (nCones + nExtraCones, 1))
        probs = np.random.rand(nCones + nExtraCones, 1)
        noise = np.random.normal(0, self.noiseVariance, (nCones + nExtraCones, 2))
        smoreo = np.hstack((onlySmoreoPoints + noise, colors, probs))
        smornn.lidarCallback(allLidarPoints)
        smornn.smoreoCallback(smoreo)
        out = smornn.run()
        colors[nCones:] = 4
        probs[nCones:] = 1
        groundTruth = np.hstack((allLidarPoints, colors.reshape(-1, 1), probs.reshape(-1, 1)))
        self.assertTrue(np.all(out == groundTruth))


if __name__ == "__main__":
    unittest.main()
