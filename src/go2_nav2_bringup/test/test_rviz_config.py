from pathlib import Path

import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config'
    / 'visual_navigation.rviz'
)


def test_visual_navigation_rviz_is_valid_yaml_with_map_fixed_frame():
    config = yaml.safe_load(CONFIG_PATH.read_text())

    assert config['Visualization Manager']['Global Options'][
        'Fixed Frame'
    ] == 'map'


def test_visual_navigation_rviz_contains_robot_and_nav2_topics():
    config_text = CONFIG_PATH.read_text()

    assert 'rviz_default_plugins/RobotModel' in config_text
    assert '/robot_description' in config_text
    assert '/rtabmap/map' in config_text
    assert '/scan' in config_text
    assert '/global_costmap/costmap' in config_text
    assert '/local_costmap/costmap' in config_text
    assert '/plan' in config_text
    assert '/local_plan' in config_text
