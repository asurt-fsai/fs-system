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
        self.params: Dict[str, Any]
        self.updateParams(params)
        self.allLandMarks = LandmarkArray()
        self.allLandMarks.header.frame_id = "flir"

    def updateParams(self, params: Dict[str, Any]) -> None:
        """
        Update params of the system from the ros parameters server

        Parameters:
        ------------
        params: Dict[str,Any]
            parameters obtained from the ros parameter server.
        Returns:
        -----------
        None
        """
        try:
            for key, value in params.items():
                if key in [
                    "cut_off_y",
                    "camera_height_from_ground",
                    "cx",
                    "cy",
                    "f",
                    "cone_height",
                ]:
                    assert isinstance(value, float)
            for key in [
                "cut_off_y",
                "camera_height_from_ground",
                "cx",
                "cy",
                "f",
                "worldCords_inCamera",
                "cone_height",
            ]:
                assert key in params
        except Exception as exp:
            errMsg = "smoreo: ensure all the required parameters are provided \n \
                       - cut_off_y \n\
                       - camera_height_from_ground \n\
                       - worldCords_inCamera \n\
                       - cx \n\
                       - cy \n\
                       - f \n\
                       - cone_height"
            raise TypeError(errMsg) from exp
        self.params = params

    def filterNearBoxes(self, bboxCy: float) -> bool:
        """
        Filter boxes that are close to the car, since their base
        will mostly be behind the car tires which will lead to
        unaccurate bounding box.

        Parameters:
        ----------
        bboxCy (float):
            y cordinate of the bbox center

        Returns:
        --------
        cutOf (bool):
            Whether to consider this bounding box or not
        """
        try:
            assert isinstance(bboxCy, float)
        except Exception as exp:
            raise TypeError("smoreo: bboxCy must be a float") from exp

        if bboxCy > self.params["cut_off_y"]:
            return False
        return True

    def addToLandmarkArray(
        self, pose: npt.NDArray[np.float32], box: npt.NDArray[np.float64]
    ) -> None:
        """
        Create landmark object for the passed cone
        and append them to a landmark array.

        Parameters:
        ----------
        pose (ndArray(1,3)):
            Predicted cone position.
        box (ndArray(6,1)):
            bounding box for predicted cone consisting
            of (h,w,cy,cx,id,type,classProb)

        Returns:
        --------
        None
        """
        if not isinstance(pose, np.ndarray) or not isinstance(box, np.ndarray):
            raise TypeError("smoreo: pose and bounding box must be numpy arrays")
        if not pose.shape == (1, 3) or not box.shape == (7,):
            raise ValueError(
                "smoreo: pose must be a 1x3 array and bounding box must be a 7x1 array"
            )
        cone = Landmark()
        cone.position.x = pose[0][0]
        cone.position.y = pose[0][1]
        cone.position.z = pose[0][2]
        cone.identifier = int(box[4])
        cone.type = int(box[5])
        cone.probability = box[6]

        self.allLandMarks.landmarks.append(cone)

    def predictWithBase(self, bboxes: npt.NDArray[np.float64]) -> LandmarkArray:
        """
        Uses the bounding boxes bases to estimate a
        3d position for every box by projecting their base on the ground.

        parameters
        ----------
        bboxes: ndArray
            bounding boxes found in image with shape => #boxes x 6 (#boxes,h,w,cy,cx,id,classProb)

        Returns
        ------
        array: LandmarkArray
            3d position per bounding box
        """
        poses = []
        self.allLandMarks.landmarks = []
        cameraHeight = self.params["camera_height_from_ground"]
        for box in bboxes:
            bboxH, _, bboxCy, bboxCx, _, _, _ = box
            if self.filterNearBoxes(float(bboxCy)):
                x = bboxCx - self.params["cx"]
                yBottom = bboxCy + bboxH // 2 - self.params["cy"]
                tOne = cameraHeight / (yBottom + 1e-6)
                baseProjection = np.asarray(
                    [x * tOne, yBottom * tOne, self.params["f"] * tOne]
                ).reshape(1, 3)
                pose = baseProjection
                pose = self.params["worldCords_inCamera"].T @ pose.T
                pose = pose.reshape(1, 3)
                self.addToLandmarkArray(pose, box)
                poses.append(pose.reshape(1, 3))

        return self.allLandMarks

    def predictWithTop(self, bboxes: npt.NDArray[np.float64]) -> LandmarkArray:
        """
        Uses the bounding boxes to estimate a
        3d position for every box by projecting their top
        on the physical cone height.

        parameters
        ----------
        bboxes: ndArray
            bounding boxes found in image with shape => #boxes x 5 (#boxes,h,w,cy,cx,id)
        Returns
        ------
        array: LandmarkArray
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
