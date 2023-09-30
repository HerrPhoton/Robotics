from setuptools import find_packages, setup

package_name = 'action_turtle_commands'

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
    description='The action that control the Turtlesim',
    license='Freeware',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'action_turtle_server = action_turtle_commands.action_server:main',
        	'action_turtle_client = action_turtle_commands.action_client:main',
        ],
    },
)
