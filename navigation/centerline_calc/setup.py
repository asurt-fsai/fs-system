"""
Setup file for the lqr package.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["centerline_calc"], package_dir={"": "src"})

setup(**d)
