import os
from glob import glob

from setuptools import setup

package_name = 'go2_rtabmap_launch'

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
    maintainer='cvr',
    maintainer_email='cvr@example.com',
    description='Launch and configuration package for Go2 RTAB-Map LiDAR SLAM.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
