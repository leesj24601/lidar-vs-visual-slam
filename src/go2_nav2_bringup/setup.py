import os
from glob import glob

from setuptools import setup


package_name = 'go2_nav2_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='cvr',
    maintainer_email='cvr@example.com',
    description='Visual RTAB-Map mapping and Nav2 mode bringup for Go2.',
    license='MIT',
    entry_points={'console_scripts': []},
)
