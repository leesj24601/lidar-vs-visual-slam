from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    bringup_share = Path(
        get_package_share_directory('go2_nav2_bringup')
    )
    visual_localization_launch = str(
        Path(get_package_share_directory('go2_rtabmap_launch'))
        / 'launch'
        / 'visual_localization.launch.py'
    )
    nav2_navigation_launch = str(
        Path(get_package_share_directory('nav2_bringup'))
        / 'launch'
        / 'navigation_launch.py'
    )
    default_nav2_params = str(
        bringup_share / 'config' / 'nav2_visual_go2.yaml'
    )
    default_rviz_config = str(
        bringup_share / 'config' / 'visual_navigation.rviz'
    )

    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgbd_topic = LaunchConfiguration('rgbd_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    show_robot_model = LaunchConfiguration('show_robot_model')
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    rviz = LaunchConfiguration('rviz')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('go2_description'),
            'urdf',
            'go2_description.urdf',
        ]),
    ])
    robot_description = ParameterValue(
        robot_description_content,
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value='',
            description='Existing visual RTAB-Map database (required).',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RealSense color image topic.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='RealSense aligned depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info matching the aligned depth image.',
        ),
        DeclareLaunchArgument(
            'rgbd_topic',
            default_value='/camera/rgbd_image',
            description='Synchronized RGB-D topic used by RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Normalized Go2 odometry topic.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start rtabmap_viz during navigation.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'show_robot_model',
            default_value='true',
            description=(
                'Publish the Go2 URDF and live joints from LowState.'
            ),
        ),
        DeclareLaunchArgument(
            'lowstate_topic',
            default_value='/lowstate',
            description='Unitree Go2 LowState topic for live joints.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the navigation RobotModel view.',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz configuration for visual navigation.',
        ),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description=(
                'Enable publishing Unitree Sport requests. Keep false for '
                'localization and planning validation.'
            ),
        ),
        DeclareLaunchArgument(
            'scan_height',
            default_value='10',
            description='Depth image rows combined into each laser scan.',
        ),
        DeclareLaunchArgument(
            'scan_range_min',
            default_value='0.30',
            description='Minimum depth scan range in meters.',
        ),
        DeclareLaunchArgument(
            'scan_range_max',
            default_value='3.00',
            description='Maximum depth scan range in meters.',
        ),
        DeclareLaunchArgument(
            'min_linear_x',
            default_value='-0.50',
            description='Sport bridge reverse speed limit in m/s.',
        ),
        DeclareLaunchArgument(
            'max_linear_x',
            default_value='1.00',
            description='Sport bridge forward speed limit in m/s.',
        ),
        DeclareLaunchArgument(
            'max_linear_y',
            default_value='0.40',
            description='Sport bridge lateral speed magnitude in m/s.',
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='1.00',
            description='Sport bridge yaw-rate magnitude in rad/s.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_timeout',
            default_value='0.30',
            description='StopMove timeout after the last velocity command.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(visual_localization_launch),
            launch_arguments={
                'database_path': database_path,
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'rgbd_topic': rgbd_topic,
                'odom_topic': odom_topic,
                'rtabmap_viz': rtabmap_viz,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='aligned_depth_to_scan',
            output='screen',
            parameters=[{
                'scan_height': ParameterValue(
                    LaunchConfiguration('scan_height'),
                    value_type=int,
                ),
                'scan_time': 0.10,
                'range_min': _float_parameter('scan_range_min'),
                'range_max': _float_parameter('scan_range_max'),
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('depth', depth_topic),
                ('depth_camera_info', camera_info_topic),
                ('scan', '/scan'),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('nav2_params_file'),
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
        Node(
            package='go2_nav2_control',
            executable='lowstate_joint_state_bridge',
            name='go2_lowstate_joint_state_bridge',
            output='screen',
            condition=IfCondition(show_robot_model),
            parameters=[{
                'lowstate_topic': lowstate_topic,
                'joint_states_topic': '/joint_states',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            condition=IfCondition(show_robot_model),
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 100.0,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='go2_navigation_rviz',
            output='screen',
            condition=IfCondition(rviz),
            arguments=[
                '-d',
                LaunchConfiguration('rviz_config_file'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='go2_nav2_control',
            executable='sport_cmd_bridge',
            name='go2_sport_cmd_bridge',
            output='screen',
            parameters=[{
                'enabled': ParameterValue(
                    LaunchConfiguration('enable_motion'),
                    value_type=bool,
                ),
                'min_linear_x': _float_parameter('min_linear_x'),
                'max_linear_x': _float_parameter('max_linear_x'),
                'max_linear_y': _float_parameter('max_linear_y'),
                'max_angular_z': _float_parameter('max_angular_z'),
                'cmd_vel_timeout': _float_parameter('cmd_vel_timeout'),
                'watchdog_period': 0.05,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
