from setuptools import setup

package_name = 'go2_rtabmap_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cvr',
    maintainer_email='cvr@example.com',
    description='Bridge Go2 LiDAR odometry and point clouds into RTAB-Map inputs.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'analyze_odom_bag = go2_rtabmap_bridge.analyze_odom_bag:main',
            'bridge_node = go2_rtabmap_bridge.bridge_node:main',
            (
                'odom_initial_alignment_tf = '
                'go2_rtabmap_bridge.odom_initial_alignment_tf:main'
            ),
            'odom_tf_bridge = go2_rtabmap_bridge.odom_tf_bridge:main',
            (
                'probe_realsense_imu = '
                'go2_rtabmap_bridge.realsense_imu_probe:main'
            ),
        ],
    },
)
