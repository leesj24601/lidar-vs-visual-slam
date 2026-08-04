from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DATABASE_REQUIRED_MESSAGE = (
    'visual_localization.launch.py requires database_path. Example: '
    'database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db'
)


def _validate_database(context):
    database_path_value = LaunchConfiguration('database_path').perform(context)
    if not database_path_value:
        raise RuntimeError(DATABASE_REQUIRED_MESSAGE)

    database_path = Path(database_path_value)
    if not database_path.is_file():
        raise RuntimeError(
            f'Visual localization database does not exist: {database_path}. '
            'Run visual_slam.launch.py first or pass database_path:=<existing rtabmap.db>.'
        )
    return [LogInfo(msg=f'Visual RTAB-Map localization database path: {database_path}')]


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
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    localization_mode = LaunchConfiguration('localization_mode')

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
            description='Robot body frame used by RTAB-Map for 6DoF visual localization.',
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
            'database_path',
            default_value='',
            description='Existing visual RTAB-Map database path for localization.',
        ),
        DeclareLaunchArgument(
            'localization_mode',
            default_value='true',
            description='Run RTAB-Map in localization mode.',
        ),
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='false',
            description='Keep the localization database by default.',
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
        OpaqueFunction(function=_validate_database),
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
                    'delete_db_on_start': delete_db_on_start,
                    'localization': localization_mode,
                    'use_sim_time': use_sim_time,
                    'frame_id': frame_id,
                    'Grid/RangeMax': depth_range_max_param,
                    'Mem/IncrementalMemory': 'false',
                    'Mem/InitWMWithAllNodes': 'true',
                    'Rtabmap/DetectionRate': '2.0',
                    'RGBD/LinearUpdate': '0.05',
                    'RGBD/AngularUpdate': '0.05',
                    'RGBD/ProximityBySpace': 'true',
                    'RGBD/ProximityOdomGuess': 'true',
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
                    'database_path': database_path,
                    'delete_db_on_start': delete_db_on_start,
                    'localization': localization_mode,
                    'use_sim_time': use_sim_time,
                    'frame_id': frame_id,
                    'Grid/RangeMax': depth_range_max_param,
                },
            ],
            remappings=[
                ('odom', odom_topic),
                ('rgbd_image', rgbd_topic),
            ],
        ),
    ])
