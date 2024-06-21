from setuptools import find_packages, setup
import os

package_name = "mrpython_pcl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), ["launch/lidar.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farida",
    maintainer_email="farida@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar= mrpython_pcl.lidar:main",
        ],
    },
)
