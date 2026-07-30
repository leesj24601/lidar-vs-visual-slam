import os
from pathlib import Path
import subprocess


SCRIPT = (
    Path(__file__).resolve().parents[3]
    / 'scripts'
    / 'unitree_realsense_456_activate.bash'
)


def test_activation_script_sources_isolated_environment_in_order(tmp_path):
    foxy_setup = tmp_path / 'foxy_setup.bash'
    workspace_setup = tmp_path / 'workspace_setup.bash'
    sdk_prefix = tmp_path / 'librealsense'
    fastdds_profile = tmp_path / 'fastdds_go2.xml'
    foxy_setup.write_text(
        'export FAKE_FOXY_SOURCED=yes\n',
        encoding='utf-8',
    )
    workspace_setup.write_text(
        'export FAKE_WORKSPACE_SOURCED=yes\n',
        encoding='utf-8',
    )

    environment = os.environ.copy()
    environment.update({
        'PATH': '/existing/bin',
        'CMAKE_PREFIX_PATH': '/existing/cmake',
        'LD_LIBRARY_PATH': '/existing/lib',
        'REALSENSE_FOXY_SETUP': str(foxy_setup),
        'REALSENSE_256_PREFIX': str(sdk_prefix),
        'REALSENSE_456_SETUP': str(workspace_setup),
        'REALSENSE_FASTDDS_PROFILE': str(fastdds_profile),
    })
    command = (
        f'source "{SCRIPT}"; '
        'printf "%s\\n" '
        '"$FAKE_FOXY_SOURCED" '
        '"$FAKE_WORKSPACE_SOURCED" '
        '"$PATH" '
        '"$CMAKE_PREFIX_PATH" '
        '"$LD_LIBRARY_PATH" '
        '"$RMW_IMPLEMENTATION" '
        '"$FASTRTPS_DEFAULT_PROFILES_FILE"'
    )

    result = subprocess.run(
        ['/bin/bash', '--noprofile', '--norc', '-c', command],
        check=True,
        capture_output=True,
        env=environment,
        text=True,
    )

    lines = result.stdout.splitlines()
    assert lines == [
        'yes',
        'yes',
        f'{sdk_prefix}/bin:/existing/bin',
        f'{sdk_prefix}:/existing/cmake',
        f'{sdk_prefix}/lib:/existing/lib',
        'rmw_fastrtps_cpp',
        str(fastdds_profile),
    ]
