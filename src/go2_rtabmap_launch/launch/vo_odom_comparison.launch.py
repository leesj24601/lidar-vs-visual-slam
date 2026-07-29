from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    go2_input_odom_topic = LaunchConfiguration('go2_input_odom_topic')
    go2_odom_topic = LaunchConfiguration('go2_odom_topic')
    vo_odom_topic = LaunchConfiguration('vo_odom_topic')
    frame_id = LaunchConfiguration('frame_id')
    comparison_frame_id = LaunchConfiguration('comparison_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    go2_sensor_time_offset = ParameterValue(
        LaunchConfiguration('go2_sensor_time_offset_sec'),
        value_type=float,
    )
    alignment_max_time_gap = ParameterValue(
        LaunchConfiguration('alignment_max_time_gap_sec'),
        value_type=float,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic used by visual odometry.',
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
            default_value='/camera/vo_compare/rgbd_image',
            description='Synchronized RGB-D topic used only for comparison.',
        ),
        DeclareLaunchArgument(
            'go2_input_odom_topic',
            default_value='/utlidar/robot_odom',
            description='Raw Go2 odometry topic.',
        ),
        DeclareLaunchArgument(
            'go2_odom_topic',
            default_value='/odom/go2',
            description='Timestamp-corrected Go2 comparison odometry.',
        ),
        DeclareLaunchArgument(
            'vo_odom_topic',
            default_value='/odom/vo',
            description='RGB-D visual odometry comparison output.',
        ),
        DeclareLaunchArgument(
            'go2_sensor_time_offset_sec',
            default_value='-0.015',
            description='Residual Go2-to-camera timestamp offset in seconds.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Robot body frame represented by both odometries.',
        ),
        DeclareLaunchArgument(
            'comparison_frame_id',
            default_value='odom_compare',
            description='Common RViz frame whose origin is both first poses.',
        ),
        DeclareLaunchArgument(
            'alignment_max_time_gap_sec',
            default_value='0.05',
            description='Maximum timestamp gap for the first pose pair.',
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
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        Node(
            package='go2_rtabmap_bridge',
            executable='odom_tf_bridge',
            name='go2_comparison_odom_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_odom_topic': go2_input_odom_topic,
                'output_odom_topic': go2_odom_topic,
                'odom_frame_id': 'go2_odom',
                'footprint_frame_id': '',
                'base_frame_id': frame_id,
                'publish_tf': False,
                'planarize_base_frame': False,
                'sensor_time_offset_sec': go2_sensor_time_offset,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='go2_rtabmap_bridge',
            executable='odom_initial_alignment_tf',
            name='vo_go2_initial_alignment_tf',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'go2_odom_topic': go2_odom_topic,
                'vo_odom_topic': vo_odom_topic,
                'comparison_frame_id': comparison_frame_id,
                'max_time_gap_sec': alignment_max_time_gap,
                'buffer_size': 200,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vo_comparison_camera_static_tf',
            output='screen',
            arguments=[
                '--x', LaunchConfiguration('camera_x'),
                '--y', LaunchConfiguration('camera_y'),
                '--z', LaunchConfiguration('camera_z'),
                '--roll', LaunchConfiguration('camera_roll'),
                '--pitch', LaunchConfiguration('camera_pitch'),
                '--yaw', LaunchConfiguration('camera_yaw'),
                '--frame-id', frame_id,
                '--child-frame-id',
                LaunchConfiguration('camera_frame_id'),
            ],
        ),
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='vo_comparison_rgbd_sync',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.03,
                'queue_size': 20,
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
            name='rgbd_vo_comparison',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'frame_id': frame_id,
                'odom_frame_id': 'vo_odom',
                'publish_tf': False,
                'subscribe_rgbd': True,
                'wait_for_transform': 0.2,
                'qos': 1,
                'qos_camera_info': 1,
                'Vis/FeatureType': '8',
                'Vis/MinInliers': '20',
                'Vis/MinDepth': '0.3',
                'Vis/MaxDepth': '4.0',
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgbd_image', rgbd_topic),
                ('odom', vo_odom_topic),
                ('odom_info', '/odom_info'),
            ],
        ),
    ])
