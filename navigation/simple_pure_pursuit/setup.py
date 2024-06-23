# pylint: disable=all
# mypy: ignore-errors
"""
Install simple pure pursuit library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["simple_pure_pursuit"], package_dir={"": "src"})

setup(**d)
