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
        probability = box.probability
        xmin = box.xmin
        ymin = box.ymin
        xmax = box.xmax
        ymax = box.ymax
        centerX = box.x_center
        centerY = box.y_center
        width = box.width
        height = box.height
        detectionId = box.detection_id
        trackId = box.track_id
        boxType = box.type

        bboxes.append(
            [
                height,
                centerY,
                centerX,
                width,
                detectionId,
                boxType,
                probability,
                xmin,
                ymin,
                xmax,
                ymax,
                trackId,
            ]
        )

    return np.asarray(bboxes)
