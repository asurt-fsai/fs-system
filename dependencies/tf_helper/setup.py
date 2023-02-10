## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
"""
Setup file to install this package
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=["tf_helper"], package_dir={"": "src"})

setup(**setup_args)
