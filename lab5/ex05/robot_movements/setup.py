from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'robot_movements'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A. Leisle',
    maintainer_email='a.leisle@g.nsu.ru',
    description='Package controlling the robot with differential drive',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "circle_movement = robot_movements.circle_movement:main",
            "snake_movement = robot_movements.snake_movement:main",
        ],
    },
)
