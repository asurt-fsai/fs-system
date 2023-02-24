"""
Moreo Base
"""
from typing import Dict, List, Any, Tuple, Union
import numpy.typing as npt
import numpy as np
from cv2 import (  # pylint: disable=no-name-in-module
    KeyPoint,
    KeyPoint_convert,
    triangulatePoints,
)
from asurt_msgs.msg import LandmarkArray, Landmark
import tf
from utils.reading import Reading  # pylint: disable=import-error
from utils.visualizer import Visualizer  # pylint: disable=import-error


class MoreoBase:
    """
    This class contains the main functionality of moreo
    """

    def __init__(self, params: Dict[str, Any], visualizer: Union[Visualizer, None]) -> None:
        """create moreo base object
        The constructor for MoreoBase class.

        Parameters:
        -----------
        params: Dict[str, Any]
            A dictionary containing the parameters for the class.
        visualizer: Union[Visualizer,None]
            Visualizer object that is used to visualize the intermediate steps of the
            system. If passed as None, no visualization will be done.
        """
        self.params = params
        self.visualizer = visualizer
        self.matrixF: npt.NDArray[np.float64]

    def calculateF(
        self,
        prevR: npt.NDArray[np.float64],
        curR: npt.NDArray[np.float64],
        relativeT: npt.NDArray[np.float64],
    ) -> None:
        """Calculate fundmental marix

        Calculate the fundemental matrix between two poses of the camera.

        Parameters:
        ------------
        prevR: npt.NDArray[np.float64]
            A rotation matrix of float64 representing the first camera orientation.
        curR: npt.NDArray[np.float64]
            A rotation matrix of float64 representing the second camera orientation.
        relativeT: npt.NDArray[np.float64]
            A Numpy array of float64 representing the relative translation.

        Returns:
            None
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

    def match(
        self,
        orgKps: List[KeyPoint],
        orgDes: npt.NDArray[np.int16],
        propKps: List[KeyPoint],
        propDes: npt.NDArray[np.int16],
    ) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:

        """Match features between two images

        Match between features in image 1 and features in image 2
        Uses the epilines to simplify the matching process by:
        1) Finding an epiline in image2 for every feature in image 1
        2) Finding the closest n features to this epiline
        3) Finding the best match between the closest features using
            euclidean distance between their descriptions

        Parameters
        ----------
        orgKps: List(Keypoint)
            list of extracted key points(cv.keyPoint objects) in image 1
        orgDes: ndarray
            array of shape (num_features, 32) containing 32 byte string descriptions
            for features passed in orgKps
        propKps: List(Keypoint)]
            list of key points(cv.keyPoint objects) proposed in image 2
        propDes: ndarray
            Array of shape (num_features, 32) containing 32 byte string descriptions
            for features passed in propKps

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
        """Calculate epilines for every passed keypoint

        Get the epilines corresponding to keypoints in an image

        Parameters
        ----------
        kps : List(Keypoint)
            list of cv.keyPoint objects
        matrixF : ndarray
            Fundamental matrix of (3,3) shape

        Returns:
        ---------
        lr: ndarray
            Array that contains epiline for every keypoint passed to the function with shape (3,n)
            where n is the number of passed keypoints
        """
        pts = self.toHomogenous(kps)
        return matrixF.T @ pts.T

    def toHomogenous(self, kps: List[KeyPoint]) -> npt.NDArray[np.float64]:
        """Transfer a kerypoin to homogenous cordinates

        Parameters
        ----------
        kps : list(Keypoint)
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
        """Calculate projection matrices for two cameras

        Calculate the realtive projection matrix for two cameras relative to the current
        camera

        Parameters:
        -----------
        prevR: ndarray (3,3)
            Rotation matrix of the previous camera as a numpy ndarray with dtype float64
        curR: ndarray (3,3)
            Rotation matrix of the current camera as a numpy ndarray with dtype float64
        relativeT: ndarray (3,1)
            The relative translation of the current camera with respect
            to the previous camera as a numpy ndarray with dtype float64

        Returns:
        --------
        Tuple of two numpy ndarrays with dtype float64, the projection matrix of
        the previous camera and the current camera each of shape (3,4)
        """

        curIX0 = np.hstack((np.identity(3) @ np.identity(3), np.identity(3) @ np.zeros((3, 1))))
        curP = self.params["worldCords_inCamera"] @ curIX0

        prevIX0 = np.hstack((prevR.T @ curR, prevR.T @ relativeT))
        prevP = self.params["worldCords_inCamera"] @ prevIX0

        return prevP, curP

    def reconstruct(  # pylint: disable=too-many-locals
        self, prevReading: Reading, curReading: Reading
    ) -> LandmarkArray:
        """Reconstruct cones position between two readings

        Reconstruct cones position between two readings using their relative
        projectetion matrices and the openCV triangulation function


        Parameters:
        ------------
        prevReading: Raading
            The Reading object representing the previous reading
        curReading: Reading
            The Reading object representing the current reading

        Returns:
        landmarks: LandmarkArray
            List of 3D position of landmarks as a LanmarkArray object
        """
        prevR = tf.transformations.quaternion_matrix(prevReading.getOrientation())[:3, :3]
        curR = tf.transformations.quaternion_matrix(curReading.getOrientation())[:3, :3]

        self.calculateF(
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

        if self.visualizer is not None:
            self.visualizer.visualizeEpilines(
                prevReading.getImage(),
                curReading.getImage(),
                allPrevKps,
                allCurKps,
                prevReading.getFeaturesPerBbox(),
                self.getEpilines(allPrevKps, self.matrixF).T,
            )
            self.visualizer.drawMatches(
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
        """Triangulate between corresponding points in two images

        Apply opencv triangulation betweeen corresponding points in previous
        and current reading. The points are normalized before triangulation

        Parameters:
        ------------
        homCurPts: ndarray (N,3)
            Points in the current image as a numpy ndarray with dtype float64
        homPrevPts: ndarray (N,3)
            Points in the previous image as a numpy ndarray with dtype float64
        prevP: ndarray (3,4)
            Projection matrix of the previous image as a numpy ndarray with dtype float64
        curP: ndarray (3,4)
            Projection matrix of the current image as a numpy ndarray with dtype float64

        Returns:
        --------
        triangulatedPoints: ndarray (N,3)
            3D points as a numpy ndarray with dtype float64
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
        """Create a landmark object from a pose and bounding box

        This method creates a Landmark object and populates its properties
        based on the given pose and bounding box information.

        Parameters:
        ------------
        pose: npt.NDArray[np.float64]
            A 3-element numpy array representing the x, y, and z position of the landmark.
        bbox: npt.NDArray[np.float64]
            A 6-element numpy array representing the bounding box information for the landmark.

        Returns:
        --------
        landmark: Landmark
            A populated Landmark object.

        """
        cone = Landmark()
        cone.position.x = pose[0]
        cone.position.y = pose[1]
        cone.position.z = pose[2]
        cone.identifier = bbox[4]

        cone.type = bbox[5]
        return cone
