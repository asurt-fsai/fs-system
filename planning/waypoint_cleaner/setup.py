"""
Install the waypoints cleaner and simple waypoints cleaner libraries
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["simple_waypoints_cleaner"],
    package_dir={"": "src"},
)

setup(**d)
