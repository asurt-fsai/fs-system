"""
Utils file for smoreo
"""
import numpy as np
import numpy.typing as npt
from asurt_msgs.msg import BoundingBoxes


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
        height = box.height
        width = box.width
        centerX = box.x_center
        centerY = box.y_center
        boxId = box.id
        boxType = box.type
        classPob = box.probability
        xmin = box.xmin
        ymin = box.ymin
        xmax = box.xmax
        ymax = box.ymax
        bboxes.append([height, width, centerY, centerX, boxId, boxType, classPob, xmin, ymin, xmax, ymax])
    return np.asarray(bboxes)