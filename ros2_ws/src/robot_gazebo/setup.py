from glob import glob
import os

from setuptools import setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masterpc',
    maintainer_email='masterpc@todo.todo',
    description='Gazebo showcase for Green Guardian',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_tf_broadcaster = robot_gazebo.ground_truth_tf_broadcaster:main',
            'scan_frame_rewriter = robot_gazebo.scan_frame_rewriter:main',
        ],
    },
)
