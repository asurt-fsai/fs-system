"""
Cone classifier module to predict if a given set of points fall on a cone or not
"""
from typing import List, Tuple, Union, Optional, Any

import numpy as np
import numpy.typing as npt
from ..helpers import SingletonMeta


class ConeClassifier(metaclass=SingletonMeta):
    """
    Given 3D points that might fall on a cone, this class predicts whether the points represent
    a cone or not, and if they fall on a cone, it returns the cone center.
    """

    def __init__(
        self, radius: float, height: float, minPoints: int, l2LossTh: float, linLossPerc: float
    ):
        """
        Parameters
        ----------
        radius: float
            Radius of the base of the cone
        height: float
            Height of the cone
        minPoints: int
            Minimum number of points to consider a cone positive (an actual cone)
        l2LossTh: float
            Maximum threshold for the MSE for fitting points to the cone
        linLossPerc: float
            MSE between the extra term and the value it's supposed to have if the points fit a cone
        """
        self.coneSize = radius**2 / height**2
        self.minPoints = minPoints
        self.l2LossTh = l2LossTh
        self.linLossPerc = linLossPerc

        # Parameter Validation
        try:
            assert type(radius) in [int, float]
            assert type(height) in [int, float]
            assert type(minPoints) in [int, float]
            assert type(l2LossTh) in [int, float]
            assert type(linLossPerc) in [int, float]
            assert radius > 0
            assert height > 0
            assert minPoints > 0 and int(minPoints) == minPoints
            assert l2LossTh > 0
            assert linLossPerc > 0
        except AssertionError as exc:
            errMsg = "ConeClassifier: Ensure all parameters positive and minPoints is an integer"
            raise TypeError(errMsg) from exc

    def isCone(
        self, points: npt.NDArray[np.float64], returnLosses: bool = False
    ) -> Tuple[List[Union[bool, float]], Optional[npt.NDArray[np.float64]]]:
        """
        Given a set of points, predict whether they fall on a cone or not.

        Parameters
        ----------
        points: np.array, shape=(num_points, 3)
            Each row contains the 3D position of a point (x, y, z)
        returnLosses: bool
            Whether to return losses in addition to the coneCenter

        Returns
        -------
        pred: list
            First element is a bool, True if cone is found, False otherwise
            If argument returnLosses is True, the list also contains
            the linearizationloss and l2 loss
        coneCenter: np.array or None
            If a cone was found, it coneCenter is returned
            (x center, y center, z top vertex of the cone)
            If no cone was found, returns None
        """
        if points.shape[0] < self.minPoints:
            return [False], None

        pred: List[Any] = [False]
        coneCenter = None
        try:
            coneParams = self.fitCone(points)
            linLoss = self.linearizationLoss(*coneParams)
            l2Loss = self.l2Loss(*coneParams[:3], points)  #  type: ignore

            if l2Loss < self.l2LossTh and linLoss < self.linLossPerc:
                coneCenter = np.array(coneParams[:3]).reshape(1, -1)
                pred = [True]

            if returnLosses:
                pred.extend([linLoss, l2Loss])
        except np.linalg.LinAlgError:
            coneParams = None
            linLoss = np.inf
            l2Loss = np.inf

        return pred, coneCenter

    def fitCone(self, points: npt.NDArray[np.float64]) -> List[float]:
        """
        Given a set of points, return cone parameters fitted

        Parameters
        ----------
        points: np.array, shape=(num_points, 3)
            Each row contains the 3D position of a point (x, y, z)

        Returns
        -------
        x0: float
            Center of the cone on the x-axis
        y0: float
            Center of the cone on the y-axis
        z0: float
            Top point of the cone on the z-axis
        e: float
            Extra constant used for linearization error
        """
        x, y, z = points.T
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        z = z.reshape(-1, 1)

        features = np.hstack((2 * x, 2 * y, -2 * z * self.coneSize, np.ones((x.shape[0], 1))))
        target = x**2 + y**2 - self.coneSize * z**2

        params = np.linalg.pinv(features) @ target

        params = params.reshape(-1)
        toReturn: List[float] = params.tolist()

        return toReturn

    def l2Loss(
        self, coneX: float, coneY: float, coneZ: float, points: npt.NDArray[np.float64]
    ) -> float:
        """
        Computes the MSE after fitting a cone (assuming no linearization error)

        Parameters
        ----------
        coneX: float
            Center of the cone on the x-axis
        coneY: float
            Center of the cone on the y-axis
        coneZ: float
            Top point of the cone on the z-axis
        points: np.array, shape=(num_points, 3)
            Each row contains the 3D position of a point (x, y, z)

        Returns
        -------
        mse: float
            MSE after fitting the cone
        """
        x, y, z = points.T
        diffX = (x - coneX) ** 2
        diffY = (y - coneY) ** 2

        pred = coneZ - np.sqrt(np.abs((diffX + diffY) / self.coneSize))

        error = (z - pred) ** 2

        mse: float = float(np.mean(error))
        return mse

    def linearizationLoss(self, coneX: float, coneY: float, coneZ: float, linTerm: float) -> float:
        """
        Computes the linearization loss as a MSE to correct term value if cone

        Parameters
        ----------
        coneX: float
            Center of the cone on the x-axis
        coneY: float
            Center of the cone on the y-axis
        coneZ: float
            Top point of the cone on the z-axis
        linTerm: float
            Extra constant used for linearization error

        Returns
        -------
        loss_perc: float
            Linearization loss as a MSE
        """
        target = self.coneSize * coneZ**2 - coneX**2 - coneY**2
        loss = (target - linTerm) ** 2
        return loss
