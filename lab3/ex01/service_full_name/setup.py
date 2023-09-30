from setuptools import find_packages, setup

package_name = 'service_full_name'

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
    description='Combines three strings with the name and returns one',
    license='Freeware',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'service_name = service_full_name.service_name:main',
        	'client_name = service_full_name.client_name:main',
        ],
    },
)
