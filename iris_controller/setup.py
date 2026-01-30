import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'iris_controller'

setup(
    name=package_name,
    version='0.0.0',
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
    description='Iris Quadrotor Controller for Isaac Sim',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = iris_controller.controller_node:main',
            'control_panel = iris_controller.control_panel:main',
        ],
    },
)
