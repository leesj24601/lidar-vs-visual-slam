from setuptools import setup


package_name = 'go2_nav2_control'


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
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='cvr',
    maintainer_email='cvr@example.com',
    description=(
        'Safe Nav2 Sport-command and live Go2 joint-state bridges.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'lowstate_joint_state_bridge = '
            'go2_nav2_control.lowstate_joint_state_bridge:main',
            'sport_cmd_bridge = go2_nav2_control.sport_cmd_bridge:main',
        ],
    },
)
