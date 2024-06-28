"""setup file for the adaptivePurepursuit package"""
from glob import glob
import os
from setuptools import find_packages, setup

PACKAGENAME = "adaptivePurepursuit"

setup(
    name=PACKAGENAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGENAME]),
        ("share/" + PACKAGENAME, ["package.xml"]),
        (os.path.join("share", PACKAGENAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGENAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mohamedalaa",
    maintainer_email="mohammed.alaa200080@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "adaptive_purepursuit = adaptive_purepursuit.adaptive_pp_node:main",
            "path_gen = adaptive_purepursuit.path_gen:main",
            "test = adaptive_purepursuit.adaptive_purepursuit:main",
        ],
    },
)
