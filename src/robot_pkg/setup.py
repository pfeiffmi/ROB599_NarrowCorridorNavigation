from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'gazebo_models'), glob(os.path.join('gazebo_models', '*.sdf'))),
        (os.path.join('share', package_name, 'gazebo_plugins'), glob(os.path.join('gazebo_plugins', '*.sdf'))),
        (os.path.join('share', package_name, 'gazebo_worlds'), glob(os.path.join('gazebo_worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike54pfeiffer',
    maintainer_email='pfeiffmi@oregonstate.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = robot_pkg.main:main',
            'drive = robot_pkg.drive_straight_until_wall:main',
            'drive_action = robot_pkg.action_server_drive_until_wall:main'
        ],
    },
)
