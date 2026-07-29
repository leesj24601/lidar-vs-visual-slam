from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VO_CONFIG = PACKAGE_ROOT / 'config' / 'rgbd_odometry_vo.yaml'
RTABMAP_CONFIG = PACKAGE_ROOT / 'config' / 'rtabmap_visual_vo.yaml'


def _parameters(path):
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


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
