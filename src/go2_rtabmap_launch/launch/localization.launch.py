from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_DATABASE_PATH = (
    '/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db'
)


def _validate_database(context):
    database_path = Path(LaunchConfiguration('database_path').perform(context))
    if not database_path.is_file():
        raise RuntimeError(
            f'Localization database does not exist: {database_path}. '
            'Run slam.launch.py first or pass database_path:=<existing rtabmap.db>.'
        )
    return [LogInfo(msg=f'RTAB-Map localization database path: {database_path}')]


def generate_launch_description():
    config_path = str(
        Path(get_package_share_directory('go2_rtabmap_launch'))
        / 'config'
        / 'rtabmap_lidar_indoor.yaml'
    )
    database_path = LaunchConfiguration('database_path')
    initial_pose = LaunchConfiguration('initial_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    optimize_max_error = LaunchConfiguration('optimize_max_error')

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=DEFAULT_DATABASE_PATH,
            description='Existing RTAB-Map database path for localization.',
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='',
            description=(
                'Optional initial pose for localization: '
                '"x y z roll pitch yaw" or "x y z qx qy qz qw".'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'optimize_max_error',
            default_value='3.0',
            description=(
                'RTAB-Map RGBD/OptimizeMaxError for localization validation. '
                'Use 0 only as a diagnostic to disable rejection.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 for ROS graph, TF, cloud, and localization inspection.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start rtabmap_viz for RTAB-Map graph/statistics inspection.',
        ),
        OpaqueFunction(function=_validate_database),
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
                    'initial_pose': initial_pose,
                    'use_sim_time': use_sim_time,
                    'Mem/IncrementalMemory': 'false',
                    'Mem/InitWMWithAllNodes': 'true',
                    'RGBD/ProximityBySpace': 'true',
                    'RGBD/ProximityOdomGuess': 'false',
                    'RGBD/ProximityPathMaxNeighbors': '1',
                    'RGBD/ProximityMaxGraphDepth': '0',
                    'RGBD/ProximityGlobalScanMap': 'false',
                    'RGBD/OptimizeMaxError': ParameterValue(
                        optimize_max_error,
                        value_type=str,
                    ),
                    'RGBD/MaxOdomCacheSize': '10',
                    'RGBD/AngularUpdate': '0.05',
                    'RGBD/LinearUpdate': '0.05',
                    'Icp/CorrespondenceRatio': '0.2',
                    'Icp/MaxCorrespondenceDistance': '1.0',
                    'Icp/OutlierRatio': '0.7',
                    'Icp/MaxTranslation': '3.0',
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
                    'initial_pose': initial_pose,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('odom', '/odom'),
                ('scan_cloud', '/scan_cloud'),
            ],
        ),
    ])
