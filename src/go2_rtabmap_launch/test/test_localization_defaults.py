from pathlib import Path


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'localization.launch.py'


def test_localization_defaults_reject_loose_false_matches():
    text = LAUNCH_FILE.read_text()

    assert "default_value='3.0'" in text
    assert "'RGBD/ProximityBySpace': 'true'" in text
    assert "'RGBD/ProximityOdomGuess': 'false'" in text
    assert "'RGBD/ProximityGlobalScanMap': 'false'" in text
    assert "'RGBD/OptimizeMaxError': ParameterValue(" in text
    assert "'RGBD/MaxOdomCacheSize': '10'" in text
    assert "'RGBD/AngularUpdate': '0.05'" in text
    assert "'RGBD/LinearUpdate': '0.05'" in text
    assert "'RGBD/ProximityPathMaxNeighbors': '1'" in text
    assert "'RGBD/ProximityMaxGraphDepth': '0'" in text
    assert "'Icp/CorrespondenceRatio': '0.2'" in text
    assert "'Icp/MaxCorrespondenceDistance': '1.0'" in text
    assert "'Icp/OutlierRatio': '0.7'" in text
    assert "'Icp/MaxTranslation': '3.0'" in text
