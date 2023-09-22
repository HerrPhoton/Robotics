from setuptools import find_packages, setup

package_name = 'text_to_cmd_vel'

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
    maintainer='Aleksandr Leisle',
    maintainer_email='a.leisle@g.nsu.ru',
    description='Allows you to control turtlesim using text commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'text_to_cmd_vel = text_to_cmd_vel.text_to_cmd_vel:main',
        ],
    },
)
