from setuptools import setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed Farhat',
    maintainer_email='mohammedfarhatwork@gmail.com',
    description='Camera driver, YOLO detection, and mission control for Green Guardian',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_node    = robot_vision.yolo_node:main',
            'mission_node = robot_vision.mission_node:main',
        ],
    },
)
