"""
Install the lidar library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["skidpad_acceleration"], package_dir={"": "src"})

setup(**d)
