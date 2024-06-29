"""importing the necessary libraries for data generation"""

from typing import Tuple, Any
import pandas as pd
import torch


class DataGenerator:
    """
    A class for generating data for waypoints.

    Attributes:
        conesData (Any): A dictionary containing the x and y coordinates of the cones.
        spline (Any): A list containing the x and y coordinates of the fitted spline curve.
    """

    def __init__(self, conesData: Any, spline: Any) -> None:
        """
        Initialize the DataGenerator class with cones_data and spline.

        Args:
            conesData (Any): A dictionary containing the x and y coordinates of the cones.
            spline (Any): A list containing the x and y coordinates of the fitted spline curve.
        """
        self.conesData = conesData
        self.spline = spline

    # generate data
    def generateData(self) -> pd.DataFrame:
        """
        Generate data for the waypoints.

        Returns:
            Tuple[pd.DataFrame, pd.DataFrame]: A tuple containing two DataFrames.
                The first DataFrame contains the source data with columns 'x', 'y', and 'color'.
                The second DataFrame contains the target data with columns 'x' and 'y'.
        """
        src = {"x": self.conesData["blue"][0], "y": self.conesData["blue"][1], "color": []}
        tgt = {"x": self.spline[0], "y": self.spline[1]}
        src["color"] = ["blue"] * len(src["x"])
        src["x"].extend(self.conesData["yellow"][0])
        src["y"].extend(self.conesData["yellow"][1])
        src["color"].extend(["yellow"] * len(self.conesData["yellow"][0]))
        src["x"].extend(self.conesData["orange"][0])
        src["y"].extend(self.conesData["orange"][1])
        src["color"].extend(["orange"] * len(self.conesData["orange"][0]))

        return pd.DataFrame(src, columns=None), pd.DataFrame(tgt, columns=None)

    def oneHotEncode(self, src: pd.DataFrame) -> pd.DataFrame:
        """
        One-hot encodes the 'color' column in the given DataFrame.

        Args:
            src (pd.DataFrame): The input DataFrame containing the 'color' column.

        Returns:
            pd.DataFrame: The DataFrame with the 'color' column one-hot encoded.
        """
        return pd.get_dummies(src, columns=["color"])

    def toTensors(self, src: Any, tgt: Any) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Converts the source and target dataframes into tensors.

        Args:
            src (Any): The source dataframe.
            tgt (Any): The target dataframe.

        Returns:
            Tuple[torch.Tensor, torch.Tensor]: A tuple containing the source and target tensors.
        """
        src = {
            column: torch.tensor(src[column].values, dtype=torch.float32) for column in src.columns
        }
        tgt = {
            column: torch.tensor(tgt[column].values, dtype=torch.float32) for column in tgt.columns
        }
        src = torch.cat(
            [
                src["x"].view(-1, 1),
                src["y"].view(-1, 1),
                src["color_blue"].view(-1, 1),
                src["color_orange"].view(-1, 1),
                src["color_yellow"].view(-1, 1),
            ],
            dim=1,
        )
        tgt = torch.cat([tgt["x"].view(-1, 1), tgt["y"].view(-1, 1)], dim=1)
        return src, tgt
