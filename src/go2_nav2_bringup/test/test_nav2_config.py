import ast
from pathlib import Path

import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'nav2_visual_go2.yaml'
)


def _parameters(config, *keys):
    value = config
    for key in keys:
        value = value[key]
    return value['ros__parameters']


def test_nav2_uses_rtabmap_localization_map_and_realsense_scan():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    global_costmap = _parameters(config, 'global_costmap', 'global_costmap')
    local_costmap = _parameters(config, 'local_costmap', 'local_costmap')
    obstacle_layer = local_costmap['obstacle_layer']

    assert 'amcl' not in config
    assert 'map_server' not in config
    assert global_costmap['global_frame'] == 'map'
    assert global_costmap['robot_base_frame'] == 'base_link'
    assert global_costmap['static_layer']['map_topic'] == '/rtabmap/map'
    assert local_costmap['global_frame'] == 'odom'
    assert local_costmap['robot_base_frame'] == 'base_link'
    assert obstacle_layer['observation_sources'] == 'depth_scan'
    assert obstacle_layer['depth_scan']['topic'] == '/scan'


def test_nav2_uses_omnidirectional_mppi_with_go2_motion_limits():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    controller_server = _parameters(config, 'controller_server')
    controller = controller_server['FollowPath']
    behavior = _parameters(config, 'behavior_server')
    velocity_smoother = _parameters(config, 'velocity_smoother')

    assert controller_server['controller_frequency'] == 20.0
    assert controller['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert controller['motion_model'] == 'Omni'
    assert controller['time_steps'] == 56
    assert controller['model_dt'] == 0.05
    assert controller['batch_size'] == 2000
    assert controller['iteration_count'] == 1
    assert controller['vx_min'] == -0.5
    assert controller['vx_max'] == 1.0
    assert controller['vy_max'] == 0.4
    assert controller['wz_max'] == 1.0
    assert controller['ax_min'] == -1.5
    assert controller['ax_max'] == 1.5
    assert controller['ay_max'] == 1.5
    assert controller['az_max'] == 2.0
    assert 'GoalAngleCritic' in controller['critics']
    assert 'RotateToGoal' not in controller['critics']
    assert velocity_smoother['max_velocity'] == [1.0, 0.4, 1.0]
    assert velocity_smoother['min_velocity'] == [-0.5, -0.4, -1.0]
    assert velocity_smoother['max_accel'] == [1.5, 1.5, 2.0]
    assert velocity_smoother['max_decel'] == [-1.5, -1.5, -2.0]
    assert behavior['max_rotational_vel'] == 1.0
    assert behavior['rotational_acc_lim'] == 2.0


def test_mppi_keeps_existing_goal_tolerances():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    goal_checker = _parameters(config, 'controller_server')[
        'general_goal_checker'
    ]

    assert goal_checker['xy_goal_tolerance'] == 0.20
    assert goal_checker['yaw_goal_tolerance'] == 0.20


def test_bringup_declares_mppi_runtime_dependency():
    package_text = (CONFIG_PATH.parents[1] / 'package.xml').read_text()

    assert '<exec_depend>nav2_mppi_controller</exec_depend>' in package_text


def test_bringup_registers_pytest_with_colcon():
    setup_text = (CONFIG_PATH.parents[1] / 'setup.py').read_text()

    assert "tests_require=['pytest']" in setup_text


def test_costmap_footprint_encloses_go2_body_with_margin():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    global_costmap = _parameters(config, 'global_costmap', 'global_costmap')
    local_costmap = _parameters(config, 'local_costmap', 'local_costmap')
    global_footprint = ast.literal_eval(global_costmap['footprint'])
    local_footprint = ast.literal_eval(local_costmap['footprint'])

    assert global_footprint == local_footprint
    assert min(point[0] for point in global_footprint) <= -0.40
    assert max(point[0] for point in global_footprint) >= 0.40
    assert min(point[1] for point in global_footprint) <= -0.21
    assert max(point[1] for point in global_footprint) >= 0.21


def test_humble_local_costmap_dimensions_use_integer_parameter_type():
    config = yaml.safe_load(CONFIG_PATH.read_text())
    local_costmap = _parameters(config, 'local_costmap', 'local_costmap')

    assert type(local_costmap['width']) is int
    assert type(local_costmap['height']) is int
