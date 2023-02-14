"""
Moreo Base
"""
from typing import Dict, List, Any, Tuple
import numpy.typing as npt
import numpy as np
from cv2 import (  # pylint: disable=no-name-in-module
    KeyPoint,
    KeyPoint_convert,
    triangulatePoints,
    drawKeypoints,
    rectangle,
    circle,
    putText,
    line,
    FONT_HERSHEY_SIMPLEX,
)
from asurt_msgs.msg import LandmarkArray, Landmark
from rospy import Publisher
import tf
from cv_bridge import CvBridge
from moreo.utils.reading import Reading  # pylint: disable=import-error


class MoreoBase:
    """
    This class contains the main functionality of moreo
    """

    def __init__(self, params: Dict[str, Any], visuals: Dict[str, Publisher]) -> None:
        """
        The constructor for MoreoBase class.

        Parameters:
            params (Dict[str, Any]): A dictionary containing the parameters for the class.
            visuals (Dict[str,Publisher]): Publishers used for visualizing intermediate outputs
        """
        self.params = params
        self.visuals = visuals
        self.matrixF: npt.NDArray[np.float64]
        self.cvBridge = CvBridge()

    def calculateF(
        self,
        prevR: npt.NDArray[np.float64],
        curR: npt.NDArray[np.float64],
        relativeT: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """
        Calculate the fundemental matrix between two poses of the camera.

        Parameters:
            prevR (npt.NDArray[np.float64]): A rotation matrix of float64 representing the
            first camera orientation.
            curR (npt.NDArray[np.float64]): A rotation matrix of float64 representing the
            second camera orientation.
            relativeT (npt.NDArray[np.float64]): A Numpy array of float64 representing the
            relative translation.

        Returns:
            NDArray[np.float64]: The calculated fundamental matrix.
        """

        relativeR = (
            self.params["worldCords_inCamera"]
            @ prevR.T
            @ curR
            @ self.params["worldCords_inCamera"].T
        )

        # Calculating the S matrix
        transX, transY, transZ, = (
            self.params["worldCords_inCamera"] @ prevR.T @ relativeT
        )
        matrixS = np.array(
            [[0, -transZ, transY], [transZ, 0, -transX], [-transY, transX, 0]],
            dtype=np.float64,
        )
        matrixE = matrixS @ relativeR
        self.matrixF = np.asarray(
            np.linalg.inv(self.params["k"]).T @ matrixE @ np.linalg.inv(self.params["k"])
        )
        print("Estimated baseline", np.sqrt(np.sum(np.power(relativeT, 2))))

        return self.matrixF

    def match(
        self,
        orgKps: List[KeyPoint],
        orgDes: npt.NDArray[np.int16],
        propKps: List[KeyPoint],
        propDes: npt.NDArray[np.int16],
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:

        """
        Match between features in image 1 and features in image 2
        Uses the epilines to simplify the matching process by:
        1) Finding an epiline in image2 for every feature in image 1
        2) Finding the closest n features to this epiline
        3) Finding the best match between the closest features using
            euclidean distance between their descriptions

        Parameters
        ----------
        orgKps : list of key points(cv.keyPoint objects) in image 1
        orgDes : ndarray of shape (num_features, 32) containing 32
        byte string descriptions for features passed in orgKps
        propKps : list of key points(cv.keyPoint objects) proposed in image 2
        propDes : ndarray of shape (num_features, 32) containing 32 byte
        string descriptions for features passed in propKps

        Returns
        -------
        matches: ndarray
        Array of shape (num_features, 3) that contains keypoints in homogenous
        cordinates that were the best match for every keypoint passed in orgKps
        errors: ndarray
        Array of the sae length of mathces, contains the error of every match calculated as
        ecluidean distance between the original key point and the matched
        """
        numberKeyPoints = 4

        epilines = self.getEpilines(orgKps, self.matrixF).T
        propPts = self.toHomogenous(propKps)
        distances = np.power(epilines @ propPts.T, 2)
        # The ijth element holds the distance^2 between point j the i epiline
        sortedIndices = np.argsort(distances, axis=1)[:, :numberKeyPoints]
        finalDes = propDes[sortedIndices]
        orgDes = orgDes[:, np.newaxis, :]
        # error = np.power((finalDes-orgDes),2)
        error = finalDes != orgDes
        assert error.shape == finalDes.shape, "Error should have a shape of #orgKps,N,32"
        # error = np.mean(error,axis=2)
        error = np.count_nonzero(error, axis=2)
        matchesIdx = np.argmin(error, axis=1).reshape(1, -1)
        errorPerMatch = np.min(error, axis=1).reshape(1, -1)
        predsIdx = sortedIndices[np.arange(matchesIdx.shape[1]), matchesIdx]
        return propPts[predsIdx][0], errorPerMatch

    def getEpilines(
        self, kps: List[KeyPoint], matrixF: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """
        Get the epilines corresponding to keypoints in an image

        Parameters
        ----------
        kps : list
        list of cv.keyPoint objects
        matrixF : ndarray
        Fundamental matrix (3,3)

        Returns:
        -------
        lr: ndarray
        Array that contains epiline for every keypoint passed to the function with shape (3,n)
        where n is the number of passed keypoints
        """
        pts = self.toHomogenous(kps)
        return matrixF.T @ pts.T

    def toHomogenous(self, kps: List[KeyPoint]) -> npt.NDArray[np.float64]:
        """
        Calculate epilines for a list of keypoints

        Parameters
        ----------
        kps : list
            list of key points(cv.keyPoint objects)

        Returns
        -------
        homogenous: ndarray
            Array that contains keypoints in homogenous cordinates with shape(n,3)
            where n is the number of passed keypoints
        """
        pts2 = np.asarray(KeyPoint_convert(kps)).reshape(-1, 2)
        ones = np.ones((len(kps), 1))
        homogenous = np.hstack((pts2, ones))
        return homogenous

    def calculateRelativeP(
        self,
        prevR: npt.NDArray[np.float64],
        curR: npt.NDArray[np.float64],
        relativeT: npt.NDArray[np.float64],
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """
        Calculate the realtive projection matrix for two cameras from the previous to the current

        Args:
        - prevR: Rotation matrix of the previous camera as a numpy ndarray with dtype float64
        - curR: Rotation matrix of the current camera as a numpy ndarray with dtype float64
        - relativeT: The relative translation of the current camera with respect
        to the previous camera
        as a numpy ndarray with dtype float64

        Returns:
        - Tuple of two numpy ndarrays with dtype float64, the projection matrix of
        the previous camera and the current camera
        """

        curIX0 = np.hstack((np.identity(3) @ np.identity(3), np.identity(3) @ np.zeros((3, 1))))
        curP = self.params["worldCords_inCamera"] @ curIX0

        prevIX0 = np.hstack((prevR.T @ curR, prevR.T @ relativeT))
        prevP = self.params["worldCords_inCamera"] @ prevIX0

        return prevP, curP

    def reconstruct(  # pylint: disable=too-many-locals
        self, prevReading: Reading, curReading: Reading
    ) -> LandmarkArray:
        """
        Reconstruct cones position between two readings

        Args:
        - prevReading: The Reading object representing the previous reading
        - curReading: The Reading object representing the current reading

        Returns:
        - List of 3D position of landmarks as a LanmarkArray object
        """
        prevR = tf.transformations.quaternion_matrix(prevReading.getOrientation())[:3, :3]
        curR = tf.transformations.quaternion_matrix(curReading.getOrientation())[:3, :3]

        matrixF = self.calculateF(
            prevR,
            curR,
            relativeT=(curReading.getPosition() - prevReading.getPosition()).reshape(3, 1),
        )

        prevP, curP = self.calculateRelativeP(
            prevR,
            curR,
            relativeT=(curReading.getPosition() - prevReading.getPosition()).reshape(3, 1),
        )

        allPrevKps: List[KeyPoint] = []
        allCurKps: List[KeyPoint] = []

        # All Points in A that was matched in image B
        targetPts: List[npt.NDArray[np.float64]] = []
        # All matched points in image B
        allMatches: List[npt.NDArray[np.float64]] = []
        landmarks = []
        # predictedPoses: List[np.ndarray] = []
        for _, boxId in enumerate(list(curReading.getFeaturesPerBbox().keys())):

            if boxId not in prevReading.getFeaturesPerBbox().keys():
                # print("Bounding box not common on both images")
                continue

            curBoxInfo, curKp, curDes = curReading.getFeaturesPerBbox()[boxId]
            _, prevKp, prevDes = prevReading.getFeaturesPerBbox()[boxId]

            allPrevKps = allPrevKps + prevKp
            allCurKps = allCurKps + curKp

            if len(prevKp) == 0 or len(curKp) == 0:
                print("No key points found")
                continue

            homCurPts, errors = self.match(prevKp, prevDes, curKp, curDes)

            homCurPts = np.asarray(homCurPts[errors[0] < 25])

            homPrevPts = self.toHomogenous(prevKp)

            homPrevPts = homPrevPts[errors[0] < 25]
            targetPts = targetPts + homPrevPts.tolist()

            allMatches = allMatches + homCurPts.tolist()

            if homPrevPts.shape[0] == 0:
                continue
            conePose = self.triangulate(homCurPts, homPrevPts, prevP, curP)
            if not np.isnan(conePose[0]):
                landmarks.append(self.getLandmark(conePose, curBoxInfo))

        self.visualizeEpilines(
            prevReading.getImage(),
            curReading.getImage(),
            allPrevKps,
            allCurKps,
            prevReading.getFeaturesPerBbox(),
            matrixF,
        )
        self.drawMatches(
            prevReading.getImage(),
            curReading.getImage(),
            np.asarray(targetPts).reshape(-1, 3),
            np.asarray(allMatches).reshape(-1, 3),
        )
        allLandmarks = LandmarkArray()
        # --------------------!!!!!! THIS
        # SHOULD ONLY BE DONE IN TESTING !!!
        # !!!------------------------
        allLandmarks.header = curReading.getImageHeader()
        allLandmarks.header.frame_id = "Obj_F"
        allLandmarks.landmarks = landmarks
        return allLandmarks

    def triangulate(
        self,
        homCurPts: npt.NDArray[np.float64],
        homPrevPts: npt.NDArray[np.float64],
        prevP: npt.NDArray[np.float64],
        curP: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """
        Triangulate points
        """
        normalizedCurPts = np.linalg.inv(self.params["k"]) @ np.asarray(homCurPts).T
        normalizedCurPts = normalizedCurPts / normalizedCurPts[2, :]
        normalizedCurPts = normalizedCurPts[:2, :]

        normalizedPrevPts = np.linalg.inv(self.params["k"]) @ np.asarray(homPrevPts).T
        normalizedPrevPts = normalizedPrevPts / normalizedPrevPts[2, :]
        normalizedPrevPts = normalizedPrevPts[:2, :]

        triangulatedPointsInHom = triangulatePoints(
            prevP,
            curP,
            np.asarray(normalizedPrevPts).reshape(2, -1),
            np.asarray(normalizedCurPts).reshape(2, -1),
        )
        triangulatedPointsInHom = np.swapaxes(triangulatedPointsInHom, 0, 1)
        triangulatedPoints = triangulatedPointsInHom[:, :3] / triangulatedPointsInHom[:, 3][:, None]
        nonNegativePoints = triangulatedPoints[triangulatedPoints[:, 0] > 0]

        conePose = np.asarray(np.median(nonNegativePoints, axis=0))
        return conePose

    def getLandmark(self, pose: npt.NDArray[np.float64], bbox: npt.NDArray[np.float64]) -> Landmark:
        """
        This method creates a Landmark object and populates its properties
        based on the given pose and bounding box information.

        Parameters:
        pose (npt.NDArray[np.float64]): A 3-element numpy array representing
        the x, y, and z position of the landmark.
        bbox (npt.NDArray[np.float64]): A 6-element numpy array representing the
        bounding box information for the landmark.

        Returns:
        Landmark: A populated Landmark object.

        """
        cone = Landmark()
        cone.position.x = pose[0]
        cone.position.y = pose[1]
        cone.position.z = pose[2]
        cone.identifier = bbox[4]

        cone.type = bbox[5]
        return cone

    def visualizeEpilines(  # pylint: disable=too-many-locals
        self,
        img1: npt.NDArray[np.float64],
        img2: npt.NDArray[np.float64],
        kp1: List[KeyPoint],
        kp2: List[KeyPoint],
        boxes: Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]],
        matrixF: npt.NDArray[np.float64],
    ) -> None:
        """
        Visualize intermediate outputs of moreo.

        Parameters:
        - img1 (npt.NDArray[np.float64]): The first image.
        - img2 (npt.NDArray[np.float64]): The second image.
        - kp1 (List[KeyPoint]): The list of keypoints in the first image.
        - kp2 (List[KeyPoint]): The list of keypoints in the second image.
        - boxes (Dict[str, Tuple[npt.NDArray[np.float64],List[KeyPoint] ,List[List[int]]]]):
            The dictionary of bounding boxes along with the keypoints and descriptors.
        - matrixF (npt.NDArray[np.float64]): The fundamental matrix.

        Returns:
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
        fig = np.hstack((visImg1, self.drawEpilines(kp1, matrixF, visImg2)))
        imageMessage = self.cvBridge.cv2_to_imgmsg(fig, encoding="passthrough")
        self.visuals["epilines"].publish(imageMessage)

    def drawEpilines(
        self,
        kp1: List[KeyPoint],
        matrixF: npt.NDArray[np.float64],
        visImg2: npt.NDArray[np.float64],
    ) -> npt.NDArray[np.float64]:
        """
        Draw epilines on second image of keypoints extracted from the first image.

        Parameters:
        - kp1 (List[KeyPoint]): The list of keypoints in the first image.
        - matrixF (npt.NDArray[np.float64]): The fundamental matrix.
        - visImg2 (npt.NDArray[np.float64]): The second image.

        Returns:
        - visImg2 (npt.NDArray[np.float64]): second image after drawing all epilines on it.
        """
        imageShape = visImg2.shape
        lines = self.getEpilines(kp1, matrixF).T
        if np.sum(lines) != 0:

            for singleLine in lines:
                _, width, _ = imageShape
                cordX0, cordY0 = map(int, [0, -(singleLine[2] / singleLine[1])])
                cordX1, cordY1 = map(
                    int,
                    [width, -((singleLine[2] + singleLine[0] * width) / singleLine[1])],
                )
                visImg2 = line(visImg2, (cordX0, cordY0), (cordX1, cordY1), 1)
        return visImg2

    def drawMatches(
        self,
        img1: npt.NDArray[np.float64],
        img2: npt.NDArray[np.float64],
        pts1: npt.NDArray[np.float64],
        matches: npt.NDArray[np.float64],
    ) -> None:
        """
        Draw matches between pairs of images.

        Parameters:
            img1 (npt.NDArray[np.float64]): First input image.
            img2 (npt.NDArray[np.float64]): Second input image.
            pts1 (npt.NDArray[np.float64]): Points from first image.
            matches (npt.NDArray[np.float64]): Points from second image, matched with `pts1`.

        Raises:
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
        self.visuals["matches"].publish(self.cvBridge.cv2_to_imgmsg(out, encoding="passthrough"))
