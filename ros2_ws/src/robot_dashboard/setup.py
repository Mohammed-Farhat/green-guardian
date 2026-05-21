from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py') + glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed Farhat',
    maintainer_email='mohammed@greenguardian.local',
    description='Green Guardian autonomous waste collection robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor_node = robot_dashboard.system_monitor_node:main',
            'bin_levels_node     = robot_dashboard.bin_levels_node:main',
            'odom_node           = robot_dashboard.odom_node:main',
        ],
    },
)
