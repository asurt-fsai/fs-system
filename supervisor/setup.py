from setuptools import find_packages, setup

package_name = 'supervisor'

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
    maintainer='yomnahashem',
    maintainer_email='yomnahashem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "staticB= supervisor.nodes.staticB:main",
            "autoDemo= supervisor.nodes.autonomus_demo:main",
            "staticAtest = supervisor.nodes.staticA:main",
            "supervisor_node = supervisor.nodes.supervisor_node:main",
            "interface = supervisor.GUI.GUIStaticMissions:main"
            
        ],
    },
)

