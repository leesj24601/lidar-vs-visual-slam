import importlib.util
import os
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / 'launch'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_nav2_bringup_test_logs')


def _load_launch(filename):
    path = LAUNCH_ROOT / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(substitution):
    return substitution.perform(LaunchContext())


def _declared_arguments(description):
    return {
        entity.name: ''.join(_text(item) for item in entity.default_value)
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _included_paths(description):
    context = LaunchContext()
    result = []
    for entity in description.entities:
        if isinstance(entity, IncludeLaunchDescription):
            source = entity.launch_description_source
            location = source._LaunchDescriptionSource__location
            result.append(perform_substitutions(context, location))
    return result


def _nodes_by_package(description):
    result = {}
    for entity in description.entities:
        if isinstance(entity, Node):
            result.setdefault(entity._Node__package, []).append(entity)
    return result


def _node_with_executable(nodes, executable):
    return next(
        node
        for node in nodes
        if node._Node__node_executable == executable
    )


def _remappings(node, arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    return {
        perform_substitutions(context, source): perform_substitutions(
            context,
            target,
        )
        for source, target in node._Node__remappings
    }


def test_mapping_mode_is_visual_mapping_only(monkeypatch):
    module = _load_launch('visual_mapping_mode.launch.py')
    monkeypatch.setattr(
        module,
        'get_package_share_directory',
        lambda package: (
            str(PACKAGE_ROOT)
            if package == 'go2_nav2_bringup'
            else str(PACKAGE_ROOT.parent / 'go2_rtabmap_launch')
        ),
    )

    description = module.generate_launch_description()
    arguments = _declared_arguments(description)
    included_paths = _included_paths(description)

    assert arguments == {
        'database_path': 'maps/visual/active/rtabmap.db',
        'reset_db': 'false',
        'rtabmap_viz': 'true',
        'use_sim_time': 'false',
    }
    assert len(included_paths) == 1
    assert included_paths[0].endswith(
        'go2_rtabmap_launch/launch/visual_slam.launch.py'
    )
    assert not any(isinstance(entity, Node) for entity in description.entities)


def test_navigation_mode_uses_visual_localization_nav2_and_safe_bridge(
    monkeypatch,
):
    module = _load_launch('visual_navigation_mode.launch.py')

    def package_share(package):
        if package == 'go2_nav2_bringup':
            return str(PACKAGE_ROOT)
        if package == 'go2_rtabmap_launch':
            return str(PACKAGE_ROOT.parent / 'go2_rtabmap_launch')
        if package == 'nav2_bringup':
            return '/opt/ros/humble/share/nav2_bringup'
        raise KeyError(package)

    monkeypatch.setattr(module, 'get_package_share_directory', package_share)

    description = module.generate_launch_description()
    arguments = _declared_arguments(description)
    included_paths = _included_paths(description)
    nodes = _nodes_by_package(description)

    assert arguments['database_path'] == ''
    assert arguments['enable_motion'] == 'false'
    assert arguments['rtabmap_viz'] == 'false'
    assert arguments['use_sim_time'] == 'false'
    assert arguments['show_robot_model'] == 'true'
    assert arguments['lowstate_topic'] == '/lowstate'
    assert arguments['rviz'] == 'true'
    assert arguments['min_linear_x'] == '-0.50'
    assert arguments['max_linear_x'] == '1.00'
    assert arguments['max_linear_y'] == '0.40'
    assert arguments['max_angular_z'] == '1.00'
    assert Path(arguments['rviz_config_file']).is_file()
    assert Path(arguments['nav2_params_file']).is_file()

    assert len(included_paths) == 2
    assert any(
        path.endswith(
            'go2_rtabmap_launch/launch/visual_localization.launch.py'
        )
        for path in included_paths
    )
    nav2_launch = (
        '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
    )
    assert nav2_launch in included_paths
    assert not any('bringup_launch.py' in path for path in included_paths)

    depth_scan = _node_with_executable(
        nodes['depthimage_to_laserscan'],
        'depthimage_to_laserscan_node',
    )
    bridge = _node_with_executable(
        nodes['go2_nav2_control'],
        'sport_cmd_bridge',
    )
    _node_with_executable(
        nodes['go2_nav2_control'],
        'lowstate_joint_state_bridge',
    )
    _node_with_executable(
        nodes['robot_state_publisher'],
        'robot_state_publisher',
    )
    _node_with_executable(nodes['rviz2'], 'rviz2')
    assert _remappings(depth_scan, arguments) == {
        'depth': '/camera/aligned_depth_to_color/image_raw',
        'depth_camera_info': '/camera/color/camera_info',
        'scan': '/scan',
    }
    context = LaunchContext()
    bridge_parameters = {
        perform_substitutions(context, name)
        if isinstance(name, tuple)
        else name
        for name in bridge._Node__parameters[0]
    }
    assert {
        'min_linear_x',
        'max_linear_x',
        'max_linear_y',
        'max_angular_z',
    }.issubset(bridge_parameters)
    assert 'nav2_amcl' not in nodes
    assert 'nav2_map_server' not in nodes


def test_navigation_mode_uses_odom_free_go2_urdf():
    launch_text = (
        LAUNCH_ROOT / 'visual_navigation_mode.launch.py'
    ).read_text()

    assert "'go2_description.urdf'" in launch_text
    assert 'go2_description.urdf.xacro' not in launch_text


def test_navigation_mode_uses_humble_composition_boolean_spelling(
    monkeypatch,
):
    module = _load_launch('visual_navigation_mode.launch.py')
    monkeypatch.setattr(
        module,
        'get_package_share_directory',
        lambda package: (
            '/opt/ros/humble/share/nav2_bringup'
            if package == 'nav2_bringup'
            else str(PACKAGE_ROOT)
        ),
    )

    description = module.generate_launch_description()
    nav2_include = next(
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
        and 'nav2_bringup/launch/navigation_launch.py'
        in _included_paths(LaunchDescription([entity]))[0]
    )
    include_arguments = dict(
        nav2_include._IncludeLaunchDescription__launch_arguments
    )

    assert include_arguments['use_composition'] == 'False'
