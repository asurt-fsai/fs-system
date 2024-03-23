"""Setup for the simple_pure_pursuit package."""
from setuptools import find_packages, setup

# pylint: disable=invalid-name
package_name = "simple_pure_pursuit"

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
            "simple_pp_node = scripts.simple_pp_node:main",
            "waypoints_node = simple_pure_pursuit.waypoints_node:main",
        ],
    },
)
