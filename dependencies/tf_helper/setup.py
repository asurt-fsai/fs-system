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
        'console_scripts': [
            "staticB= supervisor.nodes.staticB:main",
            "autoDemo= supervisor.nodes.autoDemo:main",
            "staticAtest = supervisor.nodes.staticA:main",
            "supervisor_node = supervisor.nodes.supervisor_node:main",
            "interface = supervisor.GUI.gui:main",
            "testgui = supervisor.GUI.MYstateMachine:main",
            "testCanState = supervisor.testCanState:main",
            "testVcmd = supervisor.testVcmd:main",
            "testWspeeds = supervisor.testWspeeds:main",
            "testModule = supervisor.GUI.statemachine:main",
            "testAutoDemo = supervisor.nodes.AutoDemo:main",
            "testmissionlauncher = supervisor.nodes.AutoDemo:main",
            
        ],
    },
)
