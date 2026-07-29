import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VO_CONFIG = PACKAGE_ROOT / 'config' / 'rgbd_odometry_vo.yaml'
RTABMAP_CONFIG = PACKAGE_ROOT / 'config' / 'rtabmap_visual_vo.yaml'
VO_SLAM_LAUNCH = PACKAGE_ROOT / 'launch' / 'vo_visual_slam.launch.py'


def _parameters(path):
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


def _load_launch_module():
    os.environ.setdefault(
        'ROS_LOG_DIR',
        '/tmp/go2_vo_visual_slam_test_logs',
    )
    spec = importlib.util.spec_from_file_location(
        'vo_visual_slam_launch',
        VO_SLAM_LAUNCH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE_ROOT)
    return module


def _launch_context(description):
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def _nodes_by_name(description):
    return {
        node._Node__node_name: node
        for node in description.entities
        if isinstance(node, Node)
    }


def _resolved_node(node, context):
    node._perform_substitutions(context)
    parameters = {}
    for parameter_file, _ in node._Node__expanded_parameter_arguments or []:
        content = yaml.safe_load(Path(parameter_file).read_text())
        for node_parameters in content.values():
            parameters.update(node_parameters['ros__parameters'])
    remappings = dict(node._Node__expanded_remappings or [])
    return parameters, remappings


def _resolved_arguments(node, context):
    return [
        perform_substitutions(
            context,
            normalize_to_list_of_substitutions(argument),
        )
        for argument in node._Node__arguments
    ]


def test_vo_config_pins_validated_rgbd_odometry_baseline():
    parameters = _parameters(VO_CONFIG)

    assert parameters['frame_id'] == 'base_link'
    assert parameters['odom_frame_id'] == 'vo_odom'
    assert parameters['publish_tf'] is True
    assert parameters['subscribe_rgbd'] is True
    assert parameters['wait_for_transform'] == 0.2
    assert parameters['qos'] == 1
    assert parameters['qos_camera_info'] == 1
    assert parameters['Odom/Strategy'] == '0'
    assert parameters['Odom/ResetCountdown'] == '0'
    assert parameters['Vis/FeatureType'] == '8'
    assert parameters['Vis/MinInliers'] == '20'
    assert parameters['Vis/MinDepth'] == '0.3'
    assert parameters['Vis/MaxDepth'] == '4.0'


def test_rtabmap_config_uses_vo_with_rgbd_mapping():
    parameters = _parameters(RTABMAP_CONFIG)

    assert parameters['frame_id'] == 'base_link'
    assert parameters['map_frame_id'] == 'map'
    assert 'odom_frame_id' not in parameters
    assert parameters['subscribe_rgbd'] is True
    assert parameters['subscribe_scan'] is False
    assert parameters['subscribe_scan_cloud'] is False
    assert parameters['subscribe_odom_info'] is True
    assert parameters['odom_sensor_sync'] is True
    assert parameters['Reg/Strategy'] == '0'
    assert parameters['Reg/Force3DoF'] == 'false'
    assert parameters['Grid/FromDepth'] == 'true'
    assert parameters['Grid/RangeMin'] == '0.3'
    assert parameters['Grid/RangeMax'] == '3.0'
    assert parameters['Rtabmap/DetectionRate'] == '8.0'
    assert parameters['Mem/IncrementalMemory'] == 'true'
    assert parameters['Mem/InitWMWithAllNodes'] == 'false'


def test_launch_starts_only_the_vo_visual_slam_stack():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    package_executables = {
        (node._Node__package, node._Node__node_executable)
        for node in nodes.values()
    }
    assert package_executables == {
        ('tf2_ros', 'static_transform_publisher'),
        ('rtabmap_sync', 'rgbd_sync'),
        ('rtabmap_odom', 'rgbd_odometry'),
        ('rtabmap_slam', 'rtabmap'),
        ('rtabmap_viz', 'rtabmap_viz'),
    }

    for node in nodes.values():
        parameters, remappings = _resolved_node(node, context)
        assert '/utlidar/robot_odom' not in parameters.values()
        assert '/utlidar/robot_odom' not in remappings.values()

    assert nodes['rtabmap']._Node__expanded_node_namespace == '/rtabmap_vo'
    assert (
        nodes['rtabmap_viz']._Node__expanded_node_namespace
        == '/rtabmap_vo'
    )
    assert nodes['rtabmap_viz'].condition is not None


def test_vo_owns_odometry_tf_and_rtabmap_subscribes_to_its_topics():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    vo_parameters, vo_remappings = _resolved_node(
        nodes['vo_visual_odometry'],
        context,
    )
    rtabmap_parameters, rtabmap_remappings = _resolved_node(
        nodes['rtabmap'],
        context,
    )

    assert vo_parameters['frame_id'] == 'base_link'
    assert vo_parameters['odom_frame_id'] == 'vo_odom'
    assert vo_parameters['publish_tf'] is True
    assert vo_parameters['Odom/ResetCountdown'] == '0'
    assert vo_remappings == {
        'rgbd_image': '/camera/vo_slam/rgbd_image',
        'odom': '/odom/vo',
        'odom_info': '/vo_slam/odom_info',
    }

    assert 'odom_frame_id' not in rtabmap_parameters
    assert rtabmap_parameters['frame_id'] == 'base_link'
    assert rtabmap_parameters['map_frame_id'] == 'map'
    assert rtabmap_parameters['subscribe_odom_info'] is True
    assert rtabmap_remappings == {
        'odom': '/odom/vo',
        'rgbd_image': '/camera/vo_slam/rgbd_image',
        'odom_info': '/vo_slam/odom_info',
    }


def test_rgbd_sync_feeds_one_shared_topic_to_vo_and_rtabmap():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    sync_parameters, sync_remappings = _resolved_node(
        nodes['vo_slam_rgbd_sync'],
        context,
    )
    _, vo_remappings = _resolved_node(
        nodes['vo_visual_odometry'],
        context,
    )
    _, rtabmap_remappings = _resolved_node(
        nodes['rtabmap'],
        context,
    )

    assert sync_remappings == {
        'rgb/image': '/camera/color/image_raw',
        'depth/image': '/camera/aligned_depth_to_color/image_raw',
        'rgb/camera_info': '/camera/color/camera_info',
        'rgbd_image': '/camera/vo_slam/rgbd_image',
    }
    assert 'queue_size' not in sync_parameters
    assert sync_parameters['topic_queue_size'] == 20
    assert sync_parameters['sync_queue_size'] == 20
    assert vo_remappings['rgbd_image'] == sync_remappings['rgbd_image']
    assert rtabmap_remappings['rgbd_image'] == sync_remappings['rgbd_image']


def test_static_tf_connects_robot_base_to_camera_link():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    arguments = _resolved_arguments(
        nodes['vo_slam_camera_static_tf'],
        context,
    )

    assert arguments == [
        '--x', '0.34',
        '--y', '0.0',
        '--z', '0.095',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'camera_link',
    ]


def test_vo_database_has_an_isolated_default_path():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = _launch_context(description)

    assert (
        module.DEFAULT_DATABASE_PATH
        == 'maps/visual_vo/active/rtabmap.db'
    )
    assert (
        context.launch_configurations['database_path']
        == 'maps/visual_vo/active/rtabmap.db'
    )
    assert context.launch_configurations['reset_db'] == 'false'


def test_database_reset_removes_only_selected_db_and_sidecars(tmp_path):
    module = _load_launch_module()
    database_path = tmp_path / 'rtabmap.db'
    selected_paths = [
        Path(str(database_path) + suffix)
        for suffix in ('', '-shm', '-wal', '-journal')
    ]
    unrelated = tmp_path / 'keep.db'
    for path in [*selected_paths, unrelated]:
        path.write_text('data')

    context = LaunchContext()
    context.launch_configurations['database_path'] = str(database_path)
    context.launch_configurations['reset_db'] = 'true'

    module._prepare_database(context)

    assert all(not path.exists() for path in selected_paths)
    assert unrelated.read_text() == 'data'
