"""
Setup file for tf_helper package
"""
# pylint: disable=all
from setuptools import find_packages, setup

package_name = "tf_helper"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "testing = tf_helper.statusPublisherTest:main",
            "testing2 = tf_helper.markerVizTest:main",
        ],
    },
)
