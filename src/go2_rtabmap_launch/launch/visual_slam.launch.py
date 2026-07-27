from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_DATABASE_PATH = 'maps/visual/active/rtabmap.db'


def _is_true(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _prepare_database(context):
    database_path = Path(LaunchConfiguration('database_path').perform(context))
    reset_db = _is_true(LaunchConfiguration('reset_db').perform(context))
    database_path.parent.mkdir(parents=True, exist_ok=True)

    actions = [LogInfo(msg=f'Visual RTAB-Map database path: {database_path}')]
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
        / 'rtabmap_visual_real.yaml'
    )
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_sensor_time_offset = LaunchConfiguration('odom_sensor_time_offset_sec')
    odom_sensor_time_offset_param = ParameterValue(
        odom_sensor_time_offset,
        value_type=float,
    )
    frame_id = LaunchConfiguration('frame_id')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')
    depth_range_max = LaunchConfiguration('depth_range_max')
    depth_range_max_param = ParameterValue(depth_range_max, value_type=str)
    rtabmap_detection_rate = LaunchConfiguration('rtabmap_detection_rate')
    rtabmap_detection_rate_param = ParameterValue(
        rtabmap_detection_rate,
        value_type=str,
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    planarize_odom = LaunchConfiguration('planarize_odom')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='Aligned depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='RGB camera info topic.',
        ),
        DeclareLaunchArgument(
            'rgbd_topic',
            default_value='/camera/rgbd_image',
            description='Synchronized RGB-D image topic.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Normalized odometry topic for RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'odom_sensor_time_offset_sec',
            default_value='-0.015',
            description=(
                'Residual offset added to bridge odom stamps after Go2 clock '
                'epoch correction. Negative values move odom earlier.'
            ),
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Robot body frame used by RTAB-Map for 6DoF visual SLAM.',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='RGB-D camera frame.',
        ),
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.34',
            description='Camera translation X from base_link in meters.',
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera translation Y from base_link in meters.',
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.095',
            description='Camera translation Z from base_link in meters.',
        ),
        DeclareLaunchArgument(
            'camera_roll',
            default_value='0.0',
            description='Camera roll from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.0',
            description='Camera pitch from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='0.0',
            description='Camera yaw from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'depth_range_max',
            default_value='3.0',
            description='Maximum RGB-D depth range used for RTAB-Map grid generation.',
        ),
        DeclareLaunchArgument(
            'rtabmap_detection_rate',
            default_value='8.0',
            description=(
                'RTAB-Map RGB-D processing rate in Hz. The 8 Hz experiment '
                'reduces visual input spacing while the robot turns.'
            ),
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=DEFAULT_DATABASE_PATH,
            description='Visual RTAB-Map database path.',
        ),
        DeclareLaunchArgument(
            'reset_db',
            default_value='false',
            description='Delete the selected visual RTAB-Map DB before mapping.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Start rtabmap_viz for RTAB-Map graph/statistics inspection.',
        ),
        DeclareLaunchArgument(
            'planarize_odom',
            default_value='false',
            description='Publish /odom and odom->base_link with z/roll/pitch removed.',
        ),
        OpaqueFunction(function=_prepare_database),
        Node(
            package='go2_rtabmap_bridge',
            executable='odom_tf_bridge',
            name='go2_odom_tf_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'output_odom_topic': odom_topic,
                    'odom_frame_id': 'odom',
                    'footprint_frame_id': '',
                    'base_frame_id': frame_id,
                    'publish_tf': True,
                    'planarize_base_frame': planarize_odom,
                    'sensor_time_offset_sec': odom_sensor_time_offset_param,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            output='screen',
            arguments=[
                '--x', camera_x,
                '--y', camera_y,
                '--z', camera_z,
                '--roll', camera_roll,
                '--pitch', camera_pitch,
                '--yaw', camera_yaw,
                '--frame-id', frame_id,
                '--child-frame-id', LaunchConfiguration('camera_frame_id'),
            ],
        ),
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.03,
                    'queue_size': 20,
                    'sync_queue_size': 20,
                    'qos_image': 1,
                    'qos_camera_info': 1,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('rgb/image', rgb_topic),
                ('depth/image', depth_topic),
                ('rgb/camera_info', camera_info_topic),
                ('rgbd_image', rgbd_topic),
            ],
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
                    'use_sim_time': use_sim_time,
                    'frame_id': frame_id,
                    'Grid/RangeMax': depth_range_max_param,
                    'Rtabmap/DetectionRate': rtabmap_detection_rate_param,
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
                },
            ],
            remappings=[
                ('odom', odom_topic),
                ('rgbd_image', rgbd_topic),
            ],
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
                    'use_sim_time': use_sim_time,
                    'frame_id': frame_id,
                    'Grid/RangeMax': depth_range_max_param,
                    'Rtabmap/DetectionRate': rtabmap_detection_rate_param,
                },
            ],
            remappings=[
                ('odom', odom_topic),
                ('rgbd_image', rgbd_topic),
            ],
        ),
    ])
