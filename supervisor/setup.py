import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'supervisor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share",package_name,"json"),glob("json/*.json"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yomnahashem',
    maintainer_email='yomnahashem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "staticB= supervisor.nodes.staticB:main",
            "autoDemo= supervisor.nodes.autoDemo:main",
            "staticAtest = supervisor.nodes.staticA:main",
            "supervisor_node = supervisor.nodes.supervisor_node:main",
            "interface = supervisor.GUI.gui:main",
            "testgui = supervisor.GUI.MYstateMachine:main",
            "testCanState = supervisor.testCanState:main",
            "testModule = supervisor.GUI.statemachine:main",
            "testAutoDemo = supervisor.nodes.AutoDemo:main",
            "testmissionLauncher = supervisor.Testing.test_mission_launcher:main"
            
        ],
    },
)

