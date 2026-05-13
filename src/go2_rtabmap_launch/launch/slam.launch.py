from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DATABASE_PATH = (
    '/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db'
)


def _is_true(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _prepare_database(context):
    database_path = Path(LaunchConfiguration('database_path').perform(context))
    reset_db = _is_true(LaunchConfiguration('reset_db').perform(context))
    database_path.parent.mkdir(parents=True, exist_ok=True)

    actions = [LogInfo(msg=f'RTAB-Map database path: {database_path}')]
    if not reset_db:
        return actions

    removed = []
    for suffix in ('', '-shm', '-wal', '-journal'):
        path = Path(str(database_path) + suffix)
        if path.exists():
            path.unlink()
            removed.append(str(path))

    if removed:
        actions.append(LogInfo(msg=f'reset_db=true removed: {", ".join(removed)}'))
    else:
        actions.append(LogInfo(msg='reset_db=true requested; no existing DB files found.'))
    return actions


def generate_launch_description():
    config_path = str(
        Path(get_package_share_directory('go2_rtabmap_launch'))
        / 'config'
        / 'rtabmap_lidar_indoor.yaml'
    )
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=DEFAULT_DATABASE_PATH,
            description='RTAB-Map database path.',
        ),
        DeclareLaunchArgument(
            'reset_db',
            default_value='false',
            description='Delete the selected RTAB-Map DB before starting mapping.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 for ROS graph, TF, cloud, and map inspection.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start rtabmap_viz for RTAB-Map graph/statistics inspection.',
        ),
        OpaqueFunction(function=_prepare_database),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='utlidar_static_tf',
            output='screen',
            arguments=[
                '--x', '0.28945',
                '--y', '0',
                '--z', '-0.046825',
                '--roll', '0',
                '--pitch', '2.8782',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'utlidar_lidar',
            ],
        ),
        Node(
            package='go2_rtabmap_bridge',
            executable='bridge_node',
            name='go2_rtabmap_bridge',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            emulate_tty=True,
            parameters=[
                config_path,
                {
                    'database_path': database_path,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
                },
            ],
            remappings=[
                ('odom', '/odom'),
                ('scan_cloud', '/scan_cloud'),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[
                config_path,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
            remappings=[
                ('odom', '/odom'),
                ('scan_cloud', '/scan_cloud'),
            ],
        ),
    ])
