# mrpython_pcl
A python catkin package that builds the bridge between python-pcl and ROS to easily use pcl in pyhton nodes.

## Installation

1. Install the python-pcl package
```bash
sudo apt install python3-pcl
```
I'd recommend downloading the repo for examples or just check out their [github](https://github.com/strawlab/python-pcl), [documentation](https://strawlab.github.io/python-pcl/) or [website](https://python-pcl-fork.readthedocs.io/en/rc_patches4/)..

*_note: **python-pcl** is just a small python binding to the [**pointcloud**](https://pointclouds.org/) library._*

# Important for testing
Be sure to set the following in .vscode/settings.json

```
"python.testing.cwd": "${workspaceFolder}/perception/mrpython-pcl"
```

# TODO (Minor Improvements):
1) Use rospy time difference in the tracker instead of assuming constant time differences
