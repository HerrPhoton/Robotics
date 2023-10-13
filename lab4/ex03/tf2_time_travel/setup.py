from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'tf2_time_travel'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='sanya',
    maintainer_email='a.leisle@g.nsu.ru',
    description='Moves the second turtle to the position of the first turtle with a delay',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_listener = tf2_time_travel.tf2_listener:main',
            'tf2_frame_publisher = tf2_time_travel.tf2_frame_publisher:main',
        ],
    },
)
