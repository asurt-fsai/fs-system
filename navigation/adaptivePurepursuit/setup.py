from setuptools import find_packages, setup

package_name = 'adaptivePurepursuit'

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
    maintainer='mohamedalaa',
    maintainer_email='mohammed.alaa200080@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptivePurepursuit = adaptivePurepursuit.adaptive_pp_node:main',
            'path_gen = adaptivePurepursuit.path_gen:main',
            'test = adaptivePurepursuit.adaptivePurepursuit:main'
        ],
    },
)
