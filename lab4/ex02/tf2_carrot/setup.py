from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'tf2_carrot'

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
    maintainer='Aleksandr Leisle',
    maintainer_email='a.leisle@g.nsu.ru',
    description='tf2 package that moves the second turtle after the carrot frame',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_dynamic_broadcaster = tf2_carrot.tf2_dynamic_broadcaster:main',
            'tf2_listener = tf2_carrot.tf2_listener:main',
            'tf2_frame_publisher = tf2_carrot.tf2_frame_publisher:main',
        ],
    },
)
