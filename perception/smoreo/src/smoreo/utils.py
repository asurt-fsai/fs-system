"""
Utils file for smoreo
"""
import numpy as np
import numpy.typing as npt
from darknet_ros_msgs.msg import BoundingBoxes


def processBboxes(boundingBoxes: BoundingBoxes) -> npt.NDArray[np.float64]:
    """
    Process bounding boxes objects and return
    an np array representation.

    parameters
    ----------
    boundingBoxes: BoundingBoxes
        bounding boxes found in image
    Returns
    ------
    ndarray
    #boxes x 6 (#boxes,h,w,cy,cx,id,type)
    """
    bboxes = []
    for box in boundingBoxes.bounding_boxes:
        height = box.ymax - box.ymin
        width = box.xmax - box.xmin
        centerY = (box.ymax + box.ymin) // 2
        centerX = (box.xmax + box.xmin) // 2
        boxId = box.id
        classPob = box.probability
        if box.Class == "blue_cone":
            boxType = 0
        elif box.Class == "yellow_cone":
            boxType = 1
        elif box.Class == "orange_cone":
            boxType = 2
        elif box.Class == "large_cone":
            boxType = 3
        else:
            boxType = 4
        bboxes.append([height, width, centerY, centerX, boxId, boxType, classPob])
    return np.asarray(bboxes)
