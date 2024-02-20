from setuptools import find_packages, setup

package_name = 'planning_skidpad'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='mahmoudyasser32@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'path_gen = kinematic_bicycle.path_gen:main',
        'send_cones = kinematic_bicycle.send_cones:main',
        ],
    },
)
