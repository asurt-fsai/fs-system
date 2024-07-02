"""importing the necessary libraries for generating cw tracks"""

from typing import List, Any
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


class TrackGenerator:
    """
    A class for generating tracks and cones based on waypoints.
    """

    def __init__(
        self,
    ) -> None:
        pass

    def generateTrack(self, num: int = 100) -> List[Any]:
        """
        Generate a track with random waypoints.

        Parameters:
            n (int): The number of waypoints to generate. Default is 100.

        Returns:
            List[np.ndarray]: A list containing two numpy arrays representing
            the x and y coordinates of the waypoints.
        """
        final = 2 * np.pi - np.pi / 6
        theta = np.linspace(0, final, num)
        length = np.random.uniform(0, 100, num)

        x = length * np.cos(theta)
        y = length * np.sin(theta)
        return [x, y]

    def fitSpline(self, x: Any, y: Any, mult: int = 3, per: int = 1) -> List[Any]:
        """
        Fits a spline curve to the given x and y coordinates.

        Parameters:
            x (ndarray): The x-coordinates of the data points.
            y (ndarray): The y-coordinates of the data points.
            mult (int): The multiplier for the number of points on the spline curve. Default is 3.
            per (int): The periodicity of the spline curve. Default is 1.

        Returns:
            List[ndarray]: A list containing the x and y coordinates of the fitted spline curve.
            List[np.ndarray]: A list containing the x and y coordinates of the fitted spline curve.
        """
        num = mult * len(x)
        tck, uVal, *_ = interpolate.splprep([x, y], s=0, per=per)
        uNew = np.linspace(uVal.min(), uVal.max(), num)
        x, y = interpolate.splev(uNew, tck)
        return [x, y]

    # smooth the track
    def smoothTrack(self, x: Any, y: Any, iterations: int = 2) -> List[Any]:
        """
        Smooths the given track coordinates using the moving average method.

        Parameters:
            x (np.ndarray): The x-coordinates of the track.
            y (np.ndarray): The y-coordinates of the track.
            iterations (int): The number of smoothing iterations to perform. Default is 2.

        Returns:
            List[np.ndarray]: A list containing the smoothed x-coordinates and y-coordinates.

        """
        for _ in range(iterations):
            x[1:-1] = (x[0:-2] + x[1:-1] + x[2:]) / 3
            y[1:-1] = (y[0:-2] + y[1:-1] + y[2:]) / 3
        return [x, y]

    # calculate the track length
    def _calculateTrackLength(self, x: Any, y: Any) -> float:
        """
        Calculates the length of a track based on the given x and y coordinates.

        Args:
            x (np.darray): The x-coordinates of the track points.
            y (np.darray): The y-coordinates of the track points.

        Returns:
            float: The length of the track.
        """
        length = 0
        num = len(x)
        for i in range(-1, num - 1):
            length += np.sqrt((x[i] - x[i + 1]) ** 2 + (y[i] - y[i + 1]) ** 2)
        return length

    # generate cones from track
    def calcNorm(self, x: Any, y: Any, num: int) -> Any:
        """
        Calculate the normalized vectors of the given x and y coordinates.

        Args:
            x (np.ndarray): The x coordinates.
            y (np.ndarray): The y coordinates.
            num (int): The number of coordinates.

        Returns:
            Tuple[np.ndarray, np.ndarray]: The normalized x and y vectors.
        """
        normX = np.zeros(num)
        normY = np.zeros(num)
        for i in range(num):
            if i == 0:
                normX[i] = -(y[i + 1] - y[i])
                normY[i] = x[i + 1] - x[i]
            elif i == num - 1:
                normX[i] = -(y[i] - y[i - 1])
                normY[i] = x[i] - x[i - 1]
            else:
                normX[i] = -(y[i + 1] - y[i - 1])
                normY[i] = x[i + 1] - x[i - 1]
        norm = np.sqrt(float((normX**2)) + float((normY**2)))
        normX = normX / norm
        normY = normY / norm
        return normX, normY

    def generateCones(
        self, x: Any, y: Any, trackWidth: float = 1, distanceBetweenCones: float = 1
    ) -> Any:
        """
        Generates cones along the track based on the given x and y coordinates.

        Parameters:
            x (np.ndarray): The x-coordinates of the track.
            y (np.ndarray): The y-coordinates of the track.
            trackWidth (float): The width of the track. Default is 1.
            distanceBetweenCones (float): The distance between each cone. Default is 1.

        Returns:
            dict[str, List[np.ndarray]]: A dictionary containing the coordinates of the left cones,
            right cones, and the center cones.
        """
        num = len(x)

        leftCones = self.generateConesForSide(
            x, y, trackWidth, distanceBetweenCones, num, isLeft=True
        )
        rightCones = self.generateConesForSide(
            x, y, trackWidth, distanceBetweenCones, num, isLeft=False
        )

        numberOfLeftCones = len(leftCones[0])
        numberOfRightCones = len(rightCones[0])

        print(
            f"Number of left cones: {numberOfLeftCones}, "
            f"Number of right cones: {numberOfRightCones}"
        )

        return {
            "blue": leftCones,
            "yellow": rightCones,
        }

    def generateConesForSide(
        self,
        x: Any,
        y: Any,
        trackWidth: float,
        distanceBetweenCones: float,
        num: int,
        isLeft: bool = True,
    ) -> tuple[list[Any], list[Any]]:
        """
        Generates cones for a given side of the track.

        Args:
            x (np.ndarray): Array of x-coordinates.
            y (np.ndarray): Array of y-coordinates.
            normX (np.ndarray): Array of normalized x-coordinates.
            normY (np.ndarray): Array of normalized y-coordinates.
            trackWidth (float): Width of the track.
            distanceBetweenCones (float): Distance between consecutive cones.
            isLeft (bool, optional): Flag indicating whether to generate cones
            for the left side of the track. Defaults to True.

        Returns:
            tuple[list[np.ndarray], list[np.ndarray]]: A tuple containing two lists
              - conesX and conesY.
                conesX (list[np.ndarray]): List of x-coordinates of the generated cones.
                conesY (list[np.ndarray]): List of y-coordinates of the generated cones.
        """
        normX, normY = self.calcNorm(x, y, num)
        conesX, conesY = [x[0]], [y[0]]

        for i in range(len(x)):
            if isLeft:
                trackX = x + trackWidth * normX
                trackY = y + trackWidth * normY
            else:
                trackX = x - trackWidth * normX
                trackY = y - trackWidth * normY

            dist = np.sqrt((trackX[i] - conesX[-1]) ** 2 + (trackY[i] - conesY[-1]) ** 2)
            if dist >= distanceBetweenCones:
                conesX.append(trackX[i])
                conesY.append(trackY[i])

        return conesX, conesY

    def plotTrack(self, x: float, y: float, color: str = "black") -> None:
        """
        Plot a track on a graph.

        Parameters:
        x (float): The x-coordinates of the track points.
        y (float): The y-coordinates of the track points.
        color (str): The color of the track line. Default is "black".

        Returns:
        None
        """
        plt.plot(x, y, "-", color=color)

    def plotCones(self, x: float, y: float, color: str = "blue") -> None:
        """
        Plot cones on a graph.

        Parameters:
        x (float): The x-coordinate of the cone.
        y (float): The y-coordinate of the cone.
        color (str): The color of the cone. Default is "blue".

        Returns:
        None
        """
        plt.plot(x, y, "o", color=color, linestyle="")

    def plott(self, track: Any = None, cones: Any = None) -> None:
        """
        Plot the track and cones.

        Parameters:
        track (Any): The track data. Default is None.
        cones (Any): The cones data. Default is None.

        Returns:
        None
        """
        if all(v is None for v in [track, cones]):
            print("No data to plot")
            return
        plt.figure()
        if track is not None:
            self.plotTrack(track[0], track[1])
        if cones is not None:
            for i in cones:
                self.plotCones(cones[i][0], cones[i][1], color=i)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.show()
        