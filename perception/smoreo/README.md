# Smoreo
Smoreo is a computer vision pipeline used to predict the 3d positions of cones in the camera frame. It only uses the bounding boxes extracted from YOLO.

## Behind the scenes
Smoreo exploits the fact that cones will always be on the ground in our problem FS-AI UK. So, it projects the bounding box base on the ground plane and assumes that the projection of the base is the 3d position of the cone.

![Smoreo](https://drive.google.com/uc?export=view&id=1WVj9aliuL2hKJ-hyDW1-whw-uqaY00OV)

As shown in the image the line passing the through the base center in the image plane and the cone base in the real world can be represented by parameterized line equation as $ <u*t,v*t ,f*t> $. And since ideally the line will pass through the base of the cone $(x,cameraHeight,z)$ we can solve for the parameter t by equating the two equations which gives us $t = cameraHeight/v $. we can the substiute for t in the line equation to get the x,y,z coordinates of the cone. Where:
<div align = "center">

 $x = u * cH/v$

 $y = v *cH/v$

 $z = f * cH/v$
</div>

>Please notice that the point $(u,v,f)$ is in the camera coordinate system and not the image coordinate system, so we have to transform pixels to the camera frame first.
## Dependencies
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [OpenCV](https://docs.opencv.org/3.4.3/d7/d9f/tutorial_linux_install.html)
* [Numpy](https://pypi.org/project/numpy/)
* [Matplotlib](https://pypi.org/project/matplotlib/)
## Installation
### Install ROS
Follow the instructions on the [ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS Kinetic.

### Create a catkin workspace
Follow the instructions on the [ROS Wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create a catkin workspace.

### Install OpenCV
Follow the instructions on the [OpenCV website](https://docs.opencv.org/3.4.3/d7/d9f/tutorial_linux_install.html) to install OpenCV 3.4.3. Make sure to install the dependencies and the contrib modules.

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


### Build the project
Build the package with `catkin build`.

## Usage

### Launch the pipeline
Make sure to open the smoreo_sys_flir launch file and change it with the appropriate parameters and then launch the pipeline with the following command.
```
roslaunch smoreo smoreo_sys_flir.launch
```
### Tuner
Smoreo is very sensitive to the accuracy of the calibration parameters most calibration algorithms we tried didn't yield an accurate estimate of the camera parameters, so we built a simple paramter tuner in order to tune the pipeline parameters online while the system is running.
To use the tuner make sure you set the /smoreo/hardcode_params and /smoreo/in_tuning to true in the launch file. After launching smoreo run the following command:

```
roslaunch smoreo smoreo_tuner.launch
```

*Please note that currently the tuned paramters are not saved, so manually change the parameters in the launch file after tuning the pipeline
### Subscribe to the output topic
The pipeline publishes the results to the `/perception/smoreo/detected_cones_lm`    topic. Subscribe to this topic to get the results.
*To visualize the landmark array on rviz, use can use our MarkerViz package that parse landmarks to makers that can be easily visualized in rviz.*



## License
This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.
