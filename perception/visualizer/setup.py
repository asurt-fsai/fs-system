import os
from setuptools import find_packages, setup

package_name = 'visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/visualizer_launch.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mazen',
    maintainer_email='mazenelaraby9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "visualizer_node = visualizer.visualizer_node:main"
        ],
    },
)