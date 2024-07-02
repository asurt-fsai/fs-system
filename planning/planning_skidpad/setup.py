"""Module providing a function printing python version."""
import os
from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = "planning_skidpad"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mahmoud",
    maintainer_email="mahmoudyasser32@github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "SkidPadPathPlannerNode = planning_skidpad.path_gen:main",
        ],
    },
)
