"""
Class to visualize itermidiate results of the Moreo system.
"""
from typing import Dict, List, Tuple
from rospy import Publisher
import numpy.typing as npt
import numpy as np
from cv_bridge import CvBridge
from cv2 import (  # pylint: disable=no-name-in-module
    KeyPoint,
    drawKeypoints,
    rectangle,
    circle,
    putText,
    line,
    FONT_HERSHEY_SIMPLEX,
)


class Visualizer:
    """
    Class to visualize itermidiate results of the Moreo system.
    """

    def __init__(self, visualsPublishers: Dict[str, Publisher]) -> None:
        assert "epilines" in visualsPublishers.keys(), "epilines publisher not found"
        assert "matches" in visualsPublishers.keys(), "matches publisher not found"
        self.visualsPublishers = visualsPublishers
        self.cvBridge = CvBridge()

    def visualizeEpilines(  # pylint: disable=too-many-locals
        self,
        img1: npt.NDArray[np.float64],
        img2: npt.NDArray[np.float64],
        kp1: List[KeyPoint],
        kp2: List[KeyPoint],
        boxes: Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]],
        epilines: npt.NDArray[np.float64],
    ) -> None:
        """Visuaize bounding boxes and epilines.

        The following function draws passed bounding boxes and extracted keypoints
        on the first image and visualize the epilines for the extracted keypoints on the second.
        And then publish the results to the corresponding topics.

        Parameters:
        ------------

        img1: npt.NDArray[np.float64]
            The first image.
        img2: npt.NDArray[np.float64]
            The second image.
        kp1: List[KeyPoint]
            The list of keypoints in the first image.
        kp2: List[KeyPoint]
            The list of keypoints in the second image.
        boxes: Dict[str, Tuple[npt.NDArray[np.float64],List[KeyPoint] ,List[List[int]]]]
            The dictionary of bounding boxes along every key represent the bbox id
            and the value with the keypoints and descriptors.
        eplines: npt.NDArray[np.float64]
            The epilines.

        Returns:
        --------
        None.
        """
        # draw keypoints
        visImg1 = drawKeypoints(img1, kp1, None, color=(0, 255, 0))
        visImg2 = drawKeypoints(img2, kp2, None, color=(0, 255, 0))
        for bboxId in boxes.keys():
            bbox, _, _ = boxes[bboxId]
            bboxHeight, bboxWidth, bboxY, bboxX, bId, _ = bbox

            # get rectangular diagonal points
            maskStartIdx = (int(bboxY - bboxHeight // 2), int(bboxX - bboxWidth // 2))
            maskEndIdx = (
                int(maskStartIdx[0] + bboxHeight),
                int(maskStartIdx[1] + bboxWidth),
            )

            # bounding box
            visImg1 = rectangle(
                visImg1.copy(),
                (maskStartIdx[1], maskStartIdx[0]),
                (maskEndIdx[1], maskEndIdx[0]),
                255,
            )
            visImg1 = putText(
                visImg1,
                str(bId),
                (maskStartIdx[1], maskStartIdx[0] + 20),
                FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
        # Draw epilines on second image
        fig = np.hstack((visImg1, self.drawEpilines(epilines, visImg2)))
        imageMessage = self.cvBridge.cv2_to_imgmsg(fig, encoding="passthrough")
        self.visualsPublishers["epilines"].publish(imageMessage)

    def drawEpilines(
        self,
        epilines: npt.NDArray[np.float64],
        secondImg: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """Draw eplines on second image.

        Draw epilines on second image of keypoints extracted from the first image
        and return the new image with eplines drawn on it.

        Parameters:
        -----------
        eplines: npt.NDArray[np.float64]
            The epilines.
        secondImg: npt.NDArray[np.float64]
            The second image.

        Returns:
        --------
        secondImg: npt.NDArray[np.float64]
          second image after drawing all epilines on it.
        """
        imageShape = secondImg.shape
        if np.sum(epilines) != 0:

            for singleLine in epilines:
                _, width, _ = imageShape
                cordX0, cordY0 = map(int, [0, -(singleLine[2] / singleLine[1])])
                cordX1, cordY1 = map(
                    int,
                    [width, -((singleLine[2] + singleLine[0] * width) / singleLine[1])],
                )
                secondImg = line(secondImg, (cordX0, cordY0), (cordX1, cordY1), 1)
        return secondImg

    def drawMatches(
        self,
        img1: npt.NDArray[np.float64],
        img2: npt.NDArray[np.float64],
        pts1: npt.NDArray[np.float64],
        matches: npt.NDArray[np.float64],
    ) -> None:
        """Draw matches between pair of images.

        Draw keypoints matches between pairs of images. And
        publish the result on the `matches` topic.

        Parameters:
        ------------
        img1: npt.NDArray[np.float64]
            First input image.
        img2: npt.NDArray[np.float64]
            Second input image.
        pts1: npt.NDArray[np.float64]
            Points from first image.
        matches: npt.NDArray[np.float64]
            Points from second image, matched with `pts1`.

        Raises:
        -------
            AssertionError: If `pts1.shape[0]` is not equal to `matches.shape[0]`.
        """
        assert (
            pts1.shape[0] == matches.shape[0]
        ), "Taerget points and matches should be the same size"
        # Create a new output image that concatenates the two images together
        rows1 = img1.shape[0]
        cols1 = img1.shape[1]
        rows2 = img2.shape[0]
        cols2 = img2.shape[1]
        out = np.zeros((max([rows1, rows2]), cols1 + cols2, 3), dtype="uint8")
        # Place the first image to the left
        out[:rows1, :cols1, :] = img1
        # Place the next image to the right of it
        out[:rows2, cols1 : cols1 + cols2, :] = img2
        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for i, match in enumerate(matches):
            # x - columns
            # y - rows
            cordX1, cordY1, _ = pts1[i]
            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            circle(out, (int(cordX1), int(cordY1)), 4, (255, 0, 0), 1)
            circle(out, (int(match[0]) + cols1, int(match[1])), 4, (255, 0, 0), 1)
            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            line(
                out,
                (int(cordX1), int(cordY1)),
                (int(match[0]) + cols1, int(match[1])),
                (255, 0, 0),
                1,
            )
        self.visualsPublishers["matches"].publish(
            self.cvBridge.cv2_to_imgmsg(out, encoding="passthrough")
        )
