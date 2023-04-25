"""contains dataset details"""
# mypy: ignore-errors
import os
import glob
import json

from typing import List, Dict, Union

import torch
from torch.utils.data import Dataset
from torchvision import transforms

import numpy as np
import numpy.typing as npt

import matplotlib.pyplot as plt


import imgaug as ia
import imgaug.augmenters as iaa


class KeypointDataset(Dataset):
    """keypoints dataset class"""

    def __init__(
        self,
        jsonFilesPath: str,
        imgsDir: str,
        randomRotationProb: float = 0.5,
        randomTranslationProb: float = 0.5,
        randomScaleProb: float = 0.5,
    ):
        """class constructor

        By default, this class loads the data from the given directory
        and provides transformations to the images and keypoints, resizing and
        normalizing the images and keypoints respectively.

        Parameters:
        -----------
            jsonFilesPath: str
                path of the directory containing the json files
            imgsDir: str
                path of the directory containing the images
            transform: torchvision.transforms (optional)
                transform to apply on the images
            randomRotationProb: float (optional=0.5)
                rotation probability. 0 means no rotation.
            randomHorizontalFlipProb: float (optional=0.5)
                horizontal flip probability. 0 means no horizontal flip.
        """
        self.jsonFilesPath = jsonFilesPath
        self.imgsDir = imgsDir
        self.dataList: List[Dict[str, Union[List[List[int]], int, str]]] = []
        self.randomRotationProb = randomRotationProb
        self.randomTranslationProb = randomTranslationProb
        self.randomScaleProb = randomScaleProb

        # load json filenames
        jsonFilesList = self._getJsonFileNamesInDirectory()
        for each in jsonFilesList:
            self._loadJsonFileData(each)

    def __len__(self):
        """Returns the length of the dataset.

        Returns:
        --------
            int:
                length of the dataset
        """
        return len(self.dataList)

    def __getitem__(self, idx: int):
        """
        Gets an item from the dataset.

        Parameters:
        -----------
            idx: int
                index to get
        Returns:
        --------
            Tuple[torch.Tensor]
                tuple of length=2 containing the img at index=0
                and the features vector at index=1
        """
        # extract data
        item = self.dataList[idx]
        imgFilename = item["img"]
        keypoints = item["keypoints"]

        # load and preprocess image
        img = plt.imread(os.path.join(self.imgsDir, imgFilename))
        img = np.array(img)
        height, width, _ = img.shape

        keypoints = self._transformKeypointsToImageScale(keypoints, width, height)
        img, keypoints = self._affineTransformImageAndKeypoints(
            img,
            keypoints,
            self.randomRotationProb,
            self.randomTranslationProb,
            self.randomTranslationProb,
        )

        img = torch.from_numpy(img)
        keypoints = torch.from_numpy(keypoints)

        img = self._preprocessImage(img, 80, 80)
        keypoints = self._preprocessKeypoints(keypoints, width, height)

        return img, keypoints

    def _getJsonFileNamesInDirectory(self):
        """loads all json files in the self.json_path directory

        Used to extract all json filenames in the directory given
        at self.jsonFilesPath

        Returns:
        --------
            list:
                contains all jsonfiles names in the given directory
        """
        jsonFilesList = []
        for filename in glob.glob(os.path.join(self.jsonFilesPath, "*.json")):
            jsonFilesList.append(filename)

        return jsonFilesList

    def _loadJsonFileData(self, jsonFilename: str):
        """loads data of the given jsonFile after reformatting
        it to accomodate the needed information

        Parameters:
        -----------
            jsonFilename: str
                json file name to load the data from
        """
        with open(jsonFilename, encoding="utf-8") as file:
            jsonFileDataDict = json.load(file)

        # load reformated bounding box objects
        for bboxObject in jsonFileDataDict:
            # extract needed data
            ID = bboxObject["id"]
            imgFilename = bboxObject["img"].split("-")[-1]
            keypoints = bboxObject.get("kp-1")
            if keypoints is None:
                continue

            imgWidth = int(keypoints[0]["original_width"])
            imgHeight = int(keypoints[0]["original_height"])
            keypoints = np.array([[int(each["x"]), int(each["y"])] for each in keypoints])

            self.dataList.append(
                {
                    "id": ID,
                    "img": imgFilename,
                    "img_width": imgWidth,
                    "img_height": imgHeight,
                    "keypoints": keypoints,
                    "filename": jsonFilename,
                }
            )

    def _transformKeypointsToImageScale(self, keypoints: npt.NDArray, width: int, height: int):
        keypoints = keypoints / 100
        keypoints[:, 0] = keypoints[:, 0] * width
        keypoints[:, 1] = keypoints[:, 1] * height
        return keypoints

    def _preprocessKeypoints(self, keypoints: List[List[int]], width, height):
        """performs preprocessing on the given keypoints

        Transforms the given keypoints into torch tensor and normalizes the keypoints
        using the bounding box width and height

        Parameters:
        -----------
            keypoints: List[List[int]]
                keypoints list
            width: int
                bounding box width
            height: int
                bounding box height

        Returns:
        --------
            torch.Tensor:
                preprocessed keypoints Tensor
        """
        # normalization
        keypoints[:, 0] = keypoints[:, 0] / width
        keypoints[:, 1] = keypoints[:, 1] / height

        return keypoints.view(-1, 1).squeeze(1)

    def _preprocessImage(self, img: torch.Tensor, width: int, height: int):
        """Preprocesses the given image tensor by resizing it and normalizing it.

        Parameters:
        -----------
        width : int
        The desired width of the output image tensor.
        height : int
        The desired height of the output image tensor.
        img : torch.Tensor
        The input image tensor.

        Returns:
        --------
            torch.Tensor:
            The preprocessed image tensor.
        """
        transform = transforms.Compose(
            [
                transforms.Grayscale(),
                transforms.Resize((height, width)),
                # transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )
        img = img.float().permute(2, 0, 1)
        normalizedResizeImg = transform(img / 255)
        return normalizedResizeImg

    def _affineTransformImageAndKeypoints(
        self,
        img: npt.NDArray,
        keypoints: npt.NDArray,
        rotateProb: float,
        scaleProb: float,
        translateProb: float,
    ):
        """Apply random affine transformation to the image and keypoints

        Parameters:
        -----------
            img: np.ndarray
                image to apply the transformation on
            keypoints: np.ndarray
                keypoints to apply the transformation on
            rotateProb: float
                probability of applying rotation
            scaleProb: float
                probability of applying scaling
            translateProb: float
                probability of applying translation

        Returns:
        --------
            Tuple[np.ndarray, np.ndarray]:
                transformed image and keypoints
        """
        height, width = img.shape[:2]
        probRotate = np.random.uniform(0, 1)
        probTranslate = np.random.uniform(0, 1)
        probScale = np.random.uniform(0, 1)

        rotate = np.random.uniform(-10, 10) if probRotate <= rotateProb else 0
        scale = np.random.uniform(0.5, 1.0) if probScale <= scaleProb else 1.0

        translations = [None, None]  # x,y

        if probTranslate <= translateProb:
            # translate by at most 1/2 of the image size
            translations[0] = np.random.randint(-int(scale * height / 2), int(scale * height / 2))
            translations[1] = np.random.randint(-int(scale * width / 2), int(scale * width / 2))
        else:
            translations[0] = 0
            translations[1] = 0

        seq = iaa.Sequential(
            [
                iaa.Affine(
                    rotate=rotate,
                    scale=scale,
                    translate_px={"x": translations[0], "y": translations[1]},
                )
            ]
        )

        img = seq.augment_image(img)
        keypoints = seq.augment_keypoints(
            [ia.KeypointsOnImage.from_xy_array(keypoints, shape=(height, width))]
        )[0].to_xy_array()

        keypoints = np.array(keypoints)

        return img, keypoints
