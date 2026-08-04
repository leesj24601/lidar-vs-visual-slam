from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    visual_mapping_launch = str(
        Path(get_package_share_directory('go2_rtabmap_launch'))
        / 'launch'
        / 'visual_slam.launch.py'
    )

    database_path = LaunchConfiguration('database_path')
    reset_db = LaunchConfiguration('reset_db')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value='maps/visual/active/rtabmap.db',
            description='Visual RTAB-Map database written during mapping.',
        ),
        DeclareLaunchArgument(
            'reset_db',
            default_value='false',
            description='Delete the selected visual database before mapping.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Start rtabmap_viz while mapping.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visual_mapping_launch),
            launch_arguments={
                'database_path': database_path,
                'reset_db': reset_db,
                'rtabmap_viz': rtabmap_viz,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
