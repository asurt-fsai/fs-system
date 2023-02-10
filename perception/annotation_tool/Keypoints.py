
import json
import os
import torch

class Keypoint:  
    """
    Keypoint is a class that loads the keypoints data from a json file, extracts image names, keypoints,
    original image dimensions, and normalizes the keypoints.
    """
    
    def __init__(self,json_path):
        """
        Initialize the Keypoint object with the path of the json file.
        Load the data from the json file, extract the image names, keypoints, original image dimensions,
        and normalize the keypoints.
        
        Parameters:
        json_path (str): The path to the json file.
        """
        self.json_path=json_path
        self.data=[]
        self.image_names=[]
        self.all_x_idx=torch.empty(0, dtype=torch.int16)
        self.all_y_idx=torch.empty(0, dtype=torch.int16)
        self.original_dimensions=torch.empty(0, 2, dtype=torch.int16)
        self.normalized_keypoints=torch.empty(0, 2, dtype=torch.float)
        self.__load_json()
        self.__extract_image_names()
        self.__extract_points()
        self.__normalize_coordinates()
        
    def __load_json(self):
        """
        Load the json file into the 'data' attribute.
        """
        with open(self.json_path, 'r') as file:
            self.data = json.load(file)
            
    def __extract_points(self):
        """
        Extract the x and y values of all keypoints and store them in 'all_x_idx' and 'all_y_idx' respectively.
        """
        x_idxs = []
        y_idxs = []
        for item in self.data:
            kp = item['kp-1']
            x_idx = [int(point['x']) for point in kp]
            y_idx = [int(point['y']) for point in kp]
            x_idxs.append(x_idx)
            y_idxs.append(y_idx)
        self.all_x_idx = torch.tensor(x_idxs, dtype=torch.torch.int16)
        self.all_y_idx = torch.tensor(y_idxs, dtype=torch.torch.int16)
            
    def __extract_image_names(self):
        """
        Extract the image names and store them in the 'image_names' attribute.
        """
        self.image_names =[os.path.basename(item['img']).split("_")[1] + os.path.basename(item['img']).split("_")[2]
        for item in self.data] 
        
    def __normalize_coordinates(self):
        """
        Normalize the x and y values of all keypoints and store the normalized keypoints in the 'normalized_keypoints' attribute as tensors.
        Also store the original image dimensions in the 'original_dimensions' attribute as a tensor.
        """
        original_dim=[]
        keypoints = []
        for i, obj in enumerate(self.data):
            original_width = obj['kp-1'][0]['original_width']
            original_height = obj['kp-1'][0]['original_height']
            original_dim.append((original_width, original_height))
            for kp in obj['kp-1']:
                x = int(kp['x'])/ original_width
                y = int(kp['y']) / original_height
                keypoints.append([x, y])
        self.original_dimensions=torch.tensor(original_dim, dtype=torch.int16)
        self.normalized_keypoints=torch.tensor(keypoints, dtype=torch.float)

# Define the folder path
# annotsPath = "fsoco_sample/bounding_boxes/project-2-at-2023-02-04-23-10-68dc7905.json"
# Define the coordinates for multiple boxes
# Create an instance of the dataset loader
# Keypoints = Keypoint(annotsPath)
# print(Keypoints.all_x_idx[0])
