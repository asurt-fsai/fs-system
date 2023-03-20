#!/usr/bin/python3

"""
This module is used to generate a test case for smoreo
"""
import os
from typing import Dict, Any, List
import pickle
import rosbag
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from visualization_msgs.msg import MarkerArray
from tf_helper.utils import parseLandmarks
from smoreo.utils import processBboxes
from smoreo.smoreo import Smoreo


class TestCaseGeneration:
    """
    Generate a yaml file that contains a single test case to test moreo.
    """

    def __init__(
        self, params: Dict[str, Any], bagPath: str, boundingBoxtopic: str, groundTruthTopic: str
    ):
        self.params = params
        self.bagPath = bagPath
        self.boundingBoxtopic = boundingBoxtopic
        self.groundTruthTopic = groundTruthTopic
        self.bag: rosbag.Bag = None
        self.subTopics: List[str] = []
        self.testCase: Dict[str, Any] = {}

    def fetchBag(self) -> None:
        """
        Try to fetch the bag file
        """
        try:
            self.bag = rosbag.Bag(self.bagPath)
        except Exception as exp:
            raise FileNotFoundError("bag file not found") from exp

        topics = self.bag.get_type_and_topic_info()[1].keys()
        if self.boundingBoxtopic not in topics:
            raise TypeError("bounding box topic not found in the provided bag")

        self.subTopics.append(self.boundingBoxtopic)
        if self.groundTruthTopic is not None:
            if self.groundTruthTopic not in topics:
                raise TypeError("ground truth topic not found in the provided bag")
            self.subTopics.append(self.groundTruthTopic)

    def parseMarkerArray(self, markerArray: MarkerArray) -> npt.NDArray[np.float64]:
        """
        Parse the marker array to get the cone positions
        Parameters:
        -----------
        markerArray: MarkerArray
            Marker array containing the cone positions
        Returns:
        --------
        ndarray
            #cones x 2 (#cones,x,y)
        """
        cones = []
        for marker in markerArray.markers:
            cones.append([marker.pose.position.x, marker.pose.position.y])
        return np.asarray(cones)

    def visualizePredicted(
        self, predicted: npt.NDArray[np.float64], groundTruth: npt.NDArray[np.float64]
    ) -> None:
        """
        Visualize the predicted cones and the ground truth cones

        Parameters:
        -----------
        predicted: ndarray
            #cones x 2 (#cones,x,y)
        groundTruth: ndarray
            #cones x 2 (#cones,x,y)
        Returns:
        --------
        None
        """
        nearestCritereonInX = np.bitwise_and(groundTruth[:, 0] < 15, groundTruth[:, 0] > 0)
        nearestCritereonInY = np.bitwise_and(groundTruth[:, 1] < 5, groundTruth[:, 1] > -5)
        nearestCritereon = np.bitwise_and(nearestCritereonInX, nearestCritereonInY)
        nearestGroundTruth = groundTruth[nearestCritereon]
        plt.scatter(predicted[:, 1], predicted[:, 0], c="r")
        plt.scatter(nearestGroundTruth[:, 1], nearestGroundTruth[:, 0], c="b")
        plt.show()

    def createTest(
        self,
        bboxes: npt.NDArray[np.float64],
        predictedCones: npt.NDArray[np.float64],
        groundTruth: npt.NDArray[np.float64],
    ) -> None:
        """
        Create a test case

        Parameters:
        -----------
        bboxes: ndarray
            #boxes x 6 (#boxes,h,w,cy,cx,id,type,classProb)
        predictedCones: ndarray
            #cones x 2 (#cones,x,y)
        groundTruth: ndarray
            #cones x 2 (#cones,x,y)

        Returns:
        --------
        None
        """
        self.testCase["params"] = self.params
        self.testCase["bboxes"] = bboxes
        self.testCase["predictedCones"] = predictedCones
        self.testCase["groundTruth"] = groundTruth

    def generateTestCase(self) -> None:
        """
        Generate a test case for smoreo
        """
        self.fetchBag()
        smoreo = Smoreo(self.params)
        lastGroundTruth = None
        for topic, msg, _ in self.bag.read_messages(topics=self.subTopics):
            if topic == self.boundingBoxtopic:
                bbox = processBboxes(msg)
                predictedCones = smoreo.predictWithBase(bbox)
                predictedCones = parseLandmarks(predictedCones)
                if lastGroundTruth is not None:
                    self.visualizePredicted(predictedCones, lastGroundTruth)
                    self.createTest(bbox, predictedCones, lastGroundTruth)
            elif topic == self.groundTruthTopic:
                lastGroundTruth = self.parseMarkerArray(msg)


if __name__ == """__main__""":
    PARAMS = {
        "cx": 680.0,
        "cy": 540.0,
        "f": 720.0,
        "k": np.array(
            [[720, 0.0, 720], [0.0, 720, 560], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        "worldCords_inCamera": np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=int),
        "camera_height_from_ground": 0.816,
        "cut_off_y": 900.0,
        "cone_height": 0.38,
    }

    BAG_PATH = "/media/mosameh/CEFE-EBFE/first_Ipg_moreo_inputs.bag"
    TEST_CASE_PATH = r"testCase1.pickle"

    script_dir = os.path.dirname(__file__)
    abs_file_path = os.path.join(script_dir, TEST_CASE_PATH)

    BOUNDING_BOX_TOPIC = "/darknet_ros/bounding_boxes"
    GROUND_TRUTH_TOPIC = "/ObjectList"

    testCaseGeneration = TestCaseGeneration(
        PARAMS, BAG_PATH, BOUNDING_BOX_TOPIC, GROUND_TRUTH_TOPIC
    )
    try:
        testCaseGeneration.generateTestCase()
    except KeyboardInterrupt:
        with open(abs_file_path, "wb") as file:
            pickle.dump(testCaseGeneration.testCase, file)
        print("test case saved at : ", abs_file_path)
