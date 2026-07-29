from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_DATABASE_PATH = 'maps/visual_vo/active/rtabmap.db'


def _is_true(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _prepare_database(context):
    database_path = Path(LaunchConfiguration('database_path').perform(context))
    reset_db = _is_true(LaunchConfiguration('reset_db').perform(context))
    database_path.parent.mkdir(parents=True, exist_ok=True)

    actions = [LogInfo(msg=f'VO RTAB-Map database path: {database_path}')]
    if not reset_db:
        return actions

    removed = []
    for suffix in ('', '-shm', '-wal', '-journal'):
        path = Path(str(database_path) + suffix)
        if path.exists():
            path.unlink()
            removed.append(str(path))

    if removed:
        actions.append(
            LogInfo(msg=f'reset_db=true removed: {", ".join(removed)}')
        )
    else:
        actions.append(
            LogInfo(
                msg='reset_db=true requested; no existing VO DB files found.'
            )
        )
    return actions


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('go2_rtabmap_launch')
    )
    vo_config_path = str(
        package_share / 'config' / 'rgbd_odometry_vo.yaml'
    )
    rtabmap_config_path = str(
        package_share / 'config' / 'rtabmap_visual_vo.yaml'
    )

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    vo_odom_topic = LaunchConfiguration('vo_odom_topic')
    odom_info_topic = LaunchConfiguration('odom_info_topic')
    frame_id = LaunchConfiguration('frame_id')
    vo_odom_frame_id = LaunchConfiguration('vo_odom_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')
    camera_frame_id = LaunchConfiguration('camera_frame_id')
    database_path = LaunchConfiguration('database_path')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='Depth image aligned to the RGB camera.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='RGB camera calibration topic.',
        ),
        DeclareLaunchArgument(
            'rgbd_topic',
            default_value='/camera/vo_slam/rgbd_image',
            description='Shared synchronized RGB-D topic for VO and SLAM.',
        ),
        DeclareLaunchArgument(
            'vo_odom_topic',
            default_value='/odom/vo',
            description='RGB-D visual odometry output topic.',
        ),
        DeclareLaunchArgument(
            'odom_info_topic',
            default_value='/vo_slam/odom_info',
            description='RGB-D visual odometry diagnostics topic.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Robot body frame estimated by visual odometry.',
        ),
        DeclareLaunchArgument(
            'vo_odom_frame_id',
            default_value='vo_odom',
            description='Visual odometry reference frame.',
        ),
        DeclareLaunchArgument(
            'map_frame_id',
            default_value='map',
            description='RTAB-Map global frame.',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='RealSense body frame.',
        ),
        DeclareLaunchArgument('camera_x', default_value='0.34'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.095'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'database_path',
            default_value=DEFAULT_DATABASE_PATH,
            description='VO RTAB-Map database path.',
        ),
        DeclareLaunchArgument(
            'reset_db',
            default_value='false',
            description='Delete only the selected VO database before mapping.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Start rtabmap_viz for graph inspection.',
        ),
        OpaqueFunction(function=_prepare_database),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vo_slam_camera_static_tf',
            output='screen',
            arguments=[
                '--x', LaunchConfiguration('camera_x'),
                '--y', LaunchConfiguration('camera_y'),
                '--z', LaunchConfiguration('camera_z'),
                '--roll', LaunchConfiguration('camera_roll'),
                '--pitch', LaunchConfiguration('camera_pitch'),
                '--yaw', LaunchConfiguration('camera_yaw'),
                '--frame-id', frame_id,
                '--child-frame-id', camera_frame_id,
            ],
        ),
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='vo_slam_rgbd_sync',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.03,
                'topic_queue_size': 20,
                'sync_queue_size': 20,
                'qos_image': 1,
                'qos_camera_info': 1,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('rgbd_image', rgbd_topic),
            ],
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='vo_visual_odometry',
            output='screen',
            emulate_tty=True,
            parameters=[
                vo_config_path,
                {
                    'frame_id': frame_id,
                    'odom_frame_id': vo_odom_frame_id,
                    'publish_tf': True,
                    'subscribe_rgbd': True,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('rgbd_image', rgbd_topic),
                ('odom', vo_odom_topic),
                ('odom_info', odom_info_topic),
            ],
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap_vo',
            output='screen',
            emulate_tty=True,
            parameters=[
                rtabmap_config_path,
                {
                    'database_path': database_path,
                    'frame_id': frame_id,
                    'map_frame_id': map_frame_id,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('odom', vo_odom_topic),
                ('rgbd_image', rgbd_topic),
                ('odom_info', odom_info_topic),
            ],
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap_vo',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            parameters=[
                rtabmap_config_path,
                {
                    'frame_id': frame_id,
                    'map_frame_id': map_frame_id,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('odom', vo_odom_topic),
                ('rgbd_image', rgbd_topic),
                ('odom_info', odom_info_topic),
            ],
        ),
    ])
