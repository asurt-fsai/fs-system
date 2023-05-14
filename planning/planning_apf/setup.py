"""
Install the planning apf library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["planning_apf"], package_dir={"": "src"})

setup(**d)