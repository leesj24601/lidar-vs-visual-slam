import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COMPARISON_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'vo_odom_comparison.launch.py'
)


def _load_launch_description():
    os.environ.setdefault(
        'ROS_LOG_DIR',
        '/tmp/go2_vo_odom_comparison_test_logs',
    )
    spec = importlib.util.spec_from_file_location(
        'vo_odom_comparison_launch',
        COMPARISON_LAUNCH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


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
    parameter_files = node._Node__expanded_parameter_arguments or []
    parameters = {}
    for parameter_file, _ in parameter_files:
        content = yaml.safe_load(Path(parameter_file).read_text())
        parameters.update(content['/**']['ros__parameters'])
    remappings = dict(node._Node__expanded_remappings or [])
    return parameters, remappings


def test_launch_starts_only_nodes_needed_to_generate_two_odometries():
    description = _load_launch_description()
    nodes = _nodes_by_name(description)

    package_executables = {
        (node._Node__package, node._Node__node_executable)
        for node in nodes.values()
    }

    assert package_executables == {
        ('go2_rtabmap_bridge', 'odom_tf_bridge'),
        ('go2_rtabmap_bridge', 'odom_initial_alignment_tf'),
        ('rtabmap_sync', 'rgbd_sync'),
        ('rtabmap_odom', 'rgbd_odometry'),
        ('tf2_ros', 'static_transform_publisher'),
    }


def test_odometry_outputs_are_isolated_and_do_not_publish_tf():
    description = _load_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    go2_parameters, _ = _resolved_node(
        nodes['go2_comparison_odom_bridge'],
        context,
    )
    vo_parameters, vo_remappings = _resolved_node(
        nodes['rgbd_vo_comparison'],
        context,
    )

    assert go2_parameters['input_odom_topic'] == '/utlidar/robot_odom'
    assert go2_parameters['output_odom_topic'] == '/odom/go2'
    assert go2_parameters['odom_frame_id'] == 'go2_odom'
    assert go2_parameters['publish_tf'] is False
    assert go2_parameters['sensor_time_offset_sec'] == -0.015

    assert vo_parameters['frame_id'] == 'base_link'
    assert vo_parameters['odom_frame_id'] == 'vo_odom'
    assert vo_parameters['publish_tf'] is False
    assert vo_parameters['subscribe_rgbd'] is True
    assert vo_remappings['odom'] == '/odom/vo'


def test_visual_odometry_rejects_features_outside_indoor_depth_range():
    description = _load_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    vo_parameters, _ = _resolved_node(
        nodes['rgbd_vo_comparison'],
        context,
    )

    assert vo_parameters['Vis/MinDepth'] == '0.3'
    assert vo_parameters['Vis/MaxDepth'] == '4.0'


def test_visual_odometry_publishes_diagnostics_on_explicit_topic():
    description = _load_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    _, vo_remappings = _resolved_node(
        nodes['rgbd_vo_comparison'],
        context,
    )

    assert vo_remappings['odom_info'] == '/odom_info'


def test_first_synchronized_poses_are_aligned_in_common_rviz_frame():
    description = _load_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    alignment_parameters, alignment_remappings = _resolved_node(
        nodes['vo_go2_initial_alignment_tf'],
        context,
    )

    assert alignment_parameters['go2_odom_topic'] == '/odom/go2'
    assert alignment_parameters['vo_odom_topic'] == '/odom/vo'
    assert alignment_parameters['comparison_frame_id'] == 'odom_compare'
    assert alignment_parameters['max_time_gap_sec'] == 0.05
    assert alignment_remappings == {}


def test_rgbd_sync_uses_current_realsense_topics():
    description = _load_launch_description()
    context = _launch_context(description)
    nodes = _nodes_by_name(description)

    _, sync_remappings = _resolved_node(
        nodes['vo_comparison_rgbd_sync'],
        context,
    )
    _, vo_remappings = _resolved_node(
        nodes['rgbd_vo_comparison'],
        context,
    )

    assert sync_remappings == {
        'rgb/image': '/camera/color/image_raw',
        'depth/image': '/camera/aligned_depth_to_color/image_raw',
        'rgb/camera_info': '/camera/color/camera_info',
        'rgbd_image': '/camera/vo_compare/rgbd_image',
    }
    assert (
        vo_remappings['rgbd_image']
        == '/camera/vo_compare/rgbd_image'
    )
