from setuptools import setup
import os
from glob import glob

package_name = 'robot_motors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Required by ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed Farhat',
    maintainer_email='mohammed.farhat@bau.edu.lb',
    description='Motor control and odometry bridge for Green Guardian',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_bridge = robot_motors.motor_bridge:main',
        ],
    },
)
