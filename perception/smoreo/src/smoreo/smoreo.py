"""
Smoreo class which predicts the cone position using only
information found in the bounding box extracted from yolo
"""
from typing import Dict, Any
import numpy as np
import numpy.typing as npt
from asurt_msgs.msg import LandmarkArray, Landmark


class Smoreo:
    """
    This class contains the main functionality of smoreo
    A pipeline to predict 3d positions of cones using the bounding
    boxes positions only.
    """

    def __init__(self, params: Dict[str, Any]):
        self.params = params
        self.allLandMarks = LandmarkArray()
        self.allLandMarks.header.frame_id = "flir"

    def filterNearBoxes(self, bboxCy: float) -> bool:
        """
        Filter boxes that are close to the car, since their base
        will mostly be behind the car tires which will lead to
        unaccurate bounding box.

        Parameters:
        ----------
        bboxCy (float): y cordinate of the bbox center

        Returns:
        --------
        cutOf (bool): Whether to consider this bounding box or not
        """
        if bboxCy > self.params["cut_off_y"]:
            return False
        return True

    def addToLandmarkArray(
        self, pose: npt.NDArray[np.float32], box: npt.NDArray[np.float32]
    ) -> None:
        """
        Create landmark object for the passed cone
        and append them to a landmark array.

        Parameters:
        ----------
        pose (ndArray): Predicted cone position.
        box (ndArray): bounding box for predicted cone

        Returns:
        --------
        None
        """

        cone = Landmark()
        cone.position.x = pose[0][0]
        cone.position.y = pose[0][1]
        cone.position.z = pose[0][2]
        cone.identifier = box[4]
        cone.type = box[5]
        self.allLandMarks.landmarks.append(cone)

    def predictWithBase(self, bboxes: npt.NDArray[np.float32]) -> LandmarkArray:
        """
        Uses the bounding boxes bases to estimate a
        3d position for every box by projecting their bas on the ground.

        parameters
        ----------
        bboxes2: ndArray
        bounding boxes found in image with shape => #boxes x 5 (#boxes,h,w,cy,cx,id)

        Returns
        ------
        array
        3d position per bounding box
        """
        poses = []
        self.allLandMarks = LandmarkArray()
        cameraHeight = self.params["camera_height_from_ground"]
        for box in bboxes:
            bboxH, _, bboxCy, bboxCx, _, _ = box
            if self.filterNearBoxes(bboxCy):
                x = bboxCx - self.params["cx"]
                yBottom = bboxCx + bboxH // 2 - self.params["cy"]
                tOne = cameraHeight / yBottom
                baseProjection = np.asarray(
                    [x * tOne, yBottom * tOne, self.params["f"] * tOne]
                ).reshape(1, 3)
                pose = baseProjection
                pose = self.params["worldCords_inCamera"].T @ pose.T
                pose = pose.reshape(1, 3)
                self.addToLandmarkArray(pose, box)
                poses.append(pose.reshape(1, 3))

        return self.allLandMarks

    def predictWithTop(self, bboxes: npt.NDArray[np.float32]) -> LandmarkArray:
        """
        Uses the bounding boxes to estimate a
        3d position for every box by projecting their top
        on the physical cone height.

        parameters
        ----------
        bboxes2: ndArray
        bounding boxes found in image with shape => #boxes x 5 (#boxes,h,w,cy,cx,id)
        Returns
        ------
        array
        3d position per bounding box
        """
        poses = []
        cameraHeight = self.params["camera_height_from_ground"]
        for box in bboxes:
            bboxH, _, bboxCy, bboxCx, _, _ = box
            if self.filterNearBoxes(bboxCy):
                x = bboxCx - self.params["cx"]
                yTop = bboxCy - bboxH // 2 - self.params["cy"]
                tTwo = abs((cameraHeight - 5 * self.params["cone_height"] / 6) / yTop)
                topProjection = np.asarray(
                    [x * tTwo, yTop * tTwo, self.params["f"] * tTwo]
                ).reshape(1, 3)
                pose = topProjection
                pose = self.params["worldCords_inCamera"].T @ pose.T
                pose = pose.reshape(1, 3)
                self.addToLandmarkArray(pose, box)
                poses.append(pose.reshape(1, 3))
        return self.allLandMarks
