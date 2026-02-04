from glob import glob
import os

from setuptools import setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masterpc',
    maintainer_email='mohammedfarhatwork@gmail.com',
    description='Navigation + SLAM + exploration bringup for Green Guardian',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = robot_navigation.frontier_explorer:main',
        ],
    },
)
