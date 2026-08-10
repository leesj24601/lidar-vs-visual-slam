import configparser
import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node
import setuptools
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VISUAL_SLAM_LAUNCH = PACKAGE_ROOT / 'launch' / 'visual_slam.launch.py'
VISUAL_LOCALIZATION_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'visual_localization.launch.py'
)
VISUAL_CONFIG = PACKAGE_ROOT / 'config' / 'rtabmap_visual_real.yaml'
VISUAL_GUI_CONFIG = PACKAGE_ROOT / 'config' / 'rtabmap_visual_gui.ini'
SETUP_PY = PACKAGE_ROOT / 'setup.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
DOCS_ROOT = PACKAGE_ROOT.parents[1] / 'docs'
ARCHITECTURE_DOC = DOCS_ROOT / 'ARCHITECTURE.md'
VALIDATION_DOC = DOCS_ROOT / 'VALIDATION.md'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/go2_rtabmap_launch_test_logs')


def _visual_parameters():
    return yaml.safe_load(VISUAL_CONFIG.read_text())['/**']['ros__parameters']


def _load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _rtabmap_viz_arguments(path, monkeypatch):
    module = _load_launch(path)
    monkeypatch.setattr(
        module,
        'get_package_share_directory',
        lambda package: str(PACKAGE_ROOT),
    )
    description = module.generate_launch_description()
    node = next(
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity._Node__package == 'rtabmap_viz'
        and entity._Node__node_executable == 'rtabmap_viz'
    )
    context = LaunchContext()
    return [
        perform_substitutions(
            context,
            normalize_to_list_of_substitutions(argument),
        )
        for argument in node._Node__arguments or []
    ]


def _setup_data_files(monkeypatch):
    captured = {}
    monkeypatch.chdir(PACKAGE_ROOT)
    monkeypatch.setattr(
        setuptools,
        'setup',
        lambda **kwargs: captured.update(kwargs),
    )
    spec = importlib.util.spec_from_file_location('go2_rtabmap_setup', SETUP_PY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return captured['data_files']


def _gui_settings():
    assert VISUAL_GUI_CONFIG.is_file()
    text = VISUAL_GUI_CONFIG.read_text()
    assert '[Gui]' in text.splitlines()
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    parser.read_string(text)
    assert parser.has_section('Gui')
    return parser['Gui']


def _visual_localization_rtabmap_parameters(monkeypatch):
    module = _load_launch(VISUAL_LOCALIZATION_LAUNCH)
    monkeypatch.setattr(
        module,
        'get_package_share_directory',
        lambda package: str(PACKAGE_ROOT),
    )
    description = module.generate_launch_description()
    node = next(
        entity
        for entity in description.entities
        if isinstance(entity, Node)
        and entity._Node__package == 'rtabmap_slam'
        and entity._Node__node_executable == 'rtabmap'
    )
    context = LaunchContext()
    result = {}
    for parameters in node._Node__parameters:
        if not isinstance(parameters, dict):
            continue
        for name, value in parameters.items():
            resolved_name = perform_substitutions(context, name)
            if resolved_name in {'RGBD/LinearUpdate', 'RGBD/AngularUpdate'}:
                result[resolved_name] = yaml.safe_load(
                    perform_substitutions(context, value)
                )
    return result


def test_visual_slam_launch_defaults_use_rgbd_sync_and_go2_odom_bridge():
    text = VISUAL_SLAM_LAUNCH.read_text()

    assert "executable='odom_tf_bridge'" in text
    assert "'footprint_frame_id': ''" in text
    assert "'base_frame_id': frame_id" in text
    assert "'planarize_odom'" in text
    assert "'planarize_base_frame': planarize_odom" in text
    assert "'odom_sensor_time_offset_sec',\n            default_value='-0.015'" in text
    assert "'sensor_time_offset_sec': odom_sensor_time_offset_param" in text
    assert "package='rtabmap_sync'" in text
    assert "executable='rgbd_sync'" in text
    assert "default_value='false'" in text
    assert "default_value='/camera/aligned_depth_to_color/image_raw'" in text
    assert "default_value='/camera/rgbd_image'" in text
    assert "'camera_x'" in text
    assert "default_value='0.34'" in text
    assert "'--x', camera_x" in text
    assert "'--pitch', camera_pitch" in text
    assert "'depth_range_max'" in text
    assert "ParameterValue(depth_range_max, value_type=str)" in text
    assert "'Grid/RangeMax': depth_range_max_param" in text
    assert "'rtabmap_detection_rate'" in text
    assert "'rtabmap_detection_rate',\n            default_value='8.0'" in text
    assert "ParameterValue(\n        rtabmap_detection_rate," in text
    assert "'Rtabmap/DetectionRate': rtabmap_detection_rate_param" in text
    assert "DEFAULT_DATABASE_PATH = 'maps/visual/active/rtabmap.db'" in text
    assert 'default_value=DEFAULT_DATABASE_PATH' in text


def test_visual_launches_use_project_rtabmap_viz_config(monkeypatch):
    expected = ['-d', str(VISUAL_GUI_CONFIG)]

    assert _rtabmap_viz_arguments(VISUAL_SLAM_LAUNCH, monkeypatch) == expected
    assert (
        _rtabmap_viz_arguments(VISUAL_LOCALIZATION_LAUNCH, monkeypatch)
        == expected
    )


def test_visual_gui_config_pins_clean_map_cloud_rendering():
    settings = _gui_settings()

    assert settings['General\\decimation0'] == '4'
    assert settings['General\\maxDepth0'] == '3'
    assert settings['General\\cloudVoxel'] == '0.01'
    assert settings['General\\cloudNoiseRadius'] == '0.05'
    assert settings['General\\cloudNoiseMinNeighbors'] == '5'
    assert settings['General\\ptSize0'] == '3'


def test_package_installs_visual_gui_config(monkeypatch):
    data_files = _setup_data_files(monkeypatch)
    config_files = next(
        files
        for destination, files in data_files
        if destination.endswith('/config')
    )

    assert str(VISUAL_GUI_CONFIG.relative_to(PACKAGE_ROOT)) in config_files


def test_visual_yaml_uses_rgbd_visual_mapping_defaults():
    parameters = _visual_parameters()

    assert parameters['frame_id'] == 'base_link'
    assert parameters['subscribe_rgbd'] is True
    assert parameters['subscribe_scan_cloud'] is False
    assert parameters['Reg/Strategy'] == '0'
    assert parameters['Reg/Force3DoF'] == 'false'
    assert parameters['Kp/DetectorStrategy'] == '8'
    assert parameters['Vis/FeatureType'] == '8'
    assert parameters['Vis/EstimationType'] == '1'
    assert parameters['RGBD/NeighborLinkRefining'] == 'true'
    assert parameters['RGBD/ProximityBySpace'] == 'true'
    assert parameters['RGBD/ProximityOdomGuess'] == 'false'
    assert parameters['RGBD/CreateOccupancyGrid'] == 'true'
    assert parameters['Grid/Sensor'] == '1'
    assert parameters['Grid/3D'] == 'false'
    assert parameters['Grid/RayTracing'] == 'true'
    assert 'Grid/FromDepth' not in parameters
    assert parameters['Grid/RangeMin'] == '0.3'
    assert parameters['Grid/RangeMax'] == '3.0'
    assert parameters['Grid/DepthDecimation'] == '2'
    assert parameters['Grid/CellSize'] == '0.05'
    assert parameters['Rtabmap/DetectionRate'] == '8.0'
    assert parameters['RGBD/LinearUpdate'] == '0.1'
    assert parameters['RGBD/AngularUpdate'] == '0.1'
    assert parameters['topic_queue_size'] >= 100
    assert parameters['sync_queue_size'] >= 100


def test_visual_yaml_filters_high_and_isolated_depth_obstacles():
    parameters = _visual_parameters()

    assert parameters['Grid/MaxObstacleHeight'] == '0.20'
    assert parameters['Grid/NoiseFilteringRadius'] == '0.10'
    assert parameters['Grid/NoiseFilteringMinNeighbors'] == '5'


def test_architecture_documents_visual_odom_refinement():
    text = ARCHITECTURE_DOC.read_text()

    assert '## 모듈 B: Go2 odom 기반 Visual SLAM' in text
    assert '3D-to-2D PnP' in text
    assert '`Kp/DetectorStrategy` | `8`' in text
    assert '`Vis/FeatureType` | `8`' in text
    assert '/utlidar/robot_odom' in text
    assert 'sensor_time_offset_sec=-0.015' in text
    assert '`RGBD/NeighborLinkRefining` | `true`' in text
    assert '`RGBD/ProximityBySpace` | `true`' in text
    assert '`RGBD/ProximityOdomGuess` | `false`' in text
    assert '`Rtabmap/DetectionRate` | `8.0`' in text


def test_validation_documents_visual_mapping_evidence():
    text = VALIDATION_DOC.read_text()

    assert '47.03초' in text
    assert '0.9926' in text
    assert '1 Hz → 5 Hz → 8 Hz' in text
    assert '회전이 30° 이상인 세 구간' in text
    assert '실효 처리율 **7.43 Hz**' in text
    assert '**194개 inlier**' in text
    assert '**61–83 cm / 7.9–11.9°**' in text
    assert '**16–27 cm / 1.0–2.6°**' in text
    assert 'ground truth가 아니므로' in text


def test_visual_localization_requires_existing_database_without_deleting_it():
    text = VISUAL_LOCALIZATION_LAUNCH.read_text()

    assert "default_value=''" in text
    assert "delete_db_on_start" in text
    assert "'delete_db_on_start',\n            default_value='false'" in text
    assert "'Mem/IncrementalMemory': 'false'" in text
    assert "'Mem/InitWMWithAllNodes': 'true'" in text
    assert "'Rtabmap/DetectionRate': '2.0'" in text
    assert "'RGBD/ProximityBySpace': 'true'" in text
    assert "'RGBD/ProximityOdomGuess': 'true'" in text
    assert "'odom_sensor_time_offset_sec',\n            default_value='-0.015'" in text
    assert "'sensor_time_offset_sec': odom_sensor_time_offset_param" in text
    assert "'camera_x'" in text
    assert "'--x', camera_x" in text
    assert "'--pitch', camera_pitch" in text
    assert 'requires database_path' in text


def test_visual_localization_does_not_relocalize_while_stationary(monkeypatch):
    parameters = _visual_localization_rtabmap_parameters(monkeypatch)

    assert parameters['RGBD/LinearUpdate'] == '0.05'
    assert parameters['RGBD/AngularUpdate'] == '0.05'


def test_package_declares_rtabmap_sync_dependency():
    assert '<exec_depend>rtabmap_sync</exec_depend>' in PACKAGE_XML.read_text()
