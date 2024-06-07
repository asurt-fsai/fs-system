import os
from setuptools import find_packages, setup

package_name = 'video_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/video_processor.launch'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezzedinayman',
    maintainer_email='ezz_ayman@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "video_processor = video_processor.video_processor_node:main"
        ],
    },
)
