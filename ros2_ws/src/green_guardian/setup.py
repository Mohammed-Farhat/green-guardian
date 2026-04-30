from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'green_guardian'

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
            'system_monitor_node = green_guardian.system_monitor_node:main',
            'bin_levels_node     = green_guardian.bin_levels_node:main',
            'odom_node           = green_guardian.odom_node:main',
        ],
    },
)
