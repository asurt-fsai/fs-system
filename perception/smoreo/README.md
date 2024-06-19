# Smoreo
Smoreo is a computer vision pipeline used to predict the 3d positions of cones in the camera frame. It only uses the bounding boxes extracted from YOLO.

## Behind the scenes
Smoreo exploits the fact that cones will always be on the ground in our problem FS-AI UK. So, it projects the bounding box base on the ground plane and assumes that the projection of the base is the 3d position of the cone.

![Smoreo](https://drive.google.com/uc?export=view&id=1WVj9aliuL2hKJ-hyDW1-whw-uqaY00OV)

As shown in the image the line passing through the base center in the image plane and the cone base in the real world can be represented by parameterized line equation as $<u \times t,v \times t ,f \times t> $. And since ideally the line will pass through the base of the cone $(x,cameraHeight,z)$ we can solve for the parameter t by equating the two equations which gives us $t = cameraHeight/v $. we can then substiute for t in the line equation to get the x,y,z coordinates of the cone. Where:
<div align = "center">

 $x = u \times cH/v$

 $y = v \times cH/v$

 $z = f \times cH/v$
</div>

>Please notice that the point $(u,v,f)$ is in the camera coordinate system and not the image coordinate system, so we have to transform pixels to the camera frame first.
## Dependencies
* [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu)
* [OpenCV](https://pypi.org/project/opencv-python/4.5.1.48/)
* [Numpy](https://pypi.org/project/numpy/)
* [Matplotlib](https://pypi.org/project/matplotlib/)
## Installation
### Install ROS
Follow the instructions on the [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS noetic.

### Create a catkin workspace
Follow the instructions on the [ROS Wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create a catkin workspace.

### Install OpenCV
Follow the instructions on the [OpenCV website](https://pypi.org/project/opencv-python/4.5.1.48/) to install OpenCV 4.5.1. Make sure to install the dependencies.

### Install Numpy
Follow the instructions on the [Numpy website](https://pypi.org/project/numpy/) to install Numpy.

### Install Matplotlib
Follow the instructions on the [Matplotlib website](https://pypi.org/project/matplotlib/) to install Matplotlib.

### Clone the repository
Clone the repository into the `src` folder of your catkin workspace.
```
cd path/to/catkin/workspace/src
git clone
```

## Install requirements
Install the requirements using the following command:

```
pip install -r requirements.txt
```

### Build the project
Build the package with `catkin build`.

## Usage

### Launch the pipeline
Make sure to open the smoreo_sys launch file and change it with the appropriate parameters and then launch the pipeline with the following command. You can specify which setup to run using the argument "setup_name", this is used to determine which param file is used from the config directory. By the default setup_name is "flir", so you can omit it from the command if using the flir.
```
roslaunch smoreo smoreo_sys.launch setup_name:=ipg
```
### Tuner
Smoreo is very sensitive to the accuracy of the calibration parameters most calibration algorithms we tried didn't yield an accurate estimate of the camera parameters, so we built a simple paramter tuner in order to tune the pipeline parameters online while the system is running.
To use the tuner make sure the /smoreo/hardcode_params is set to True in the smoreo_sys.launch. to run smoreo and the tuner at the same time use following command:

```
roslaunch smoreo smoreo_sys.launch setup_name:=ipg tune:=True
```

### Subscribe to the output topic
The pipeline publishes the results to the `/perception/smoreo/detected_cones_lm`    topic. Subscribe to this topic to get the results.
*To visualize the landmark array on rviz, you can subscribe to the topic `/perception/smoreo/detected_cones_markers` which has the same landmarks but in the markerArray format, that can be easily visualized on rviz.*
