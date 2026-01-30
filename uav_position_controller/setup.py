import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_position_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huojiaxi',
    maintainer_email='jiaxi.huo@outlook.com',
    description='Position controller for PX4 drones using attitude-level control via MAVSDK',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = uav_position_controller.controller_node:main',
            'control_panel = uav_position_controller.control_panel:main',
        ],
    },
)
