import os
from pathlib import Path
import subprocess


SCRIPT = (
    Path(__file__).resolve().parents[3]
    / 'scripts'
    / 'unitree_realsense_456_activate.bash'
)


def test_activation_script_preserves_existing_foxy_and_dds_environment(
    tmp_path,
):
    foxy_setup = tmp_path / 'foxy_setup.bash'
    workspace_setup = tmp_path / 'workspace_setup.bash'
    sdk_prefix = tmp_path / 'librealsense'
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
        'ROS_DISTRO': 'foxy',
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': '/existing/cyclonedds.xml',
        'FASTRTPS_DEFAULT_PROFILES_FILE': '/existing/fastdds.xml',
    })
    command = (
        f'source "{SCRIPT}"; '
        'printf "%s\\n" '
        '"${FAKE_FOXY_SOURCED-unset}" '
        '"$FAKE_WORKSPACE_SOURCED" '
        '"$PATH" '
        '"$CMAKE_PREFIX_PATH" '
        '"$LD_LIBRARY_PATH" '
        '"$RMW_IMPLEMENTATION" '
        '"$CYCLONEDDS_URI" '
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
        'unset',
        'yes',
        f'{sdk_prefix}/bin:/existing/bin',
        f'{sdk_prefix}:/existing/cmake',
        f'{sdk_prefix}/lib:/existing/lib',
        'rmw_cyclonedds_cpp',
        '/existing/cyclonedds.xml',
        '/existing/fastdds.xml',
    ]


def test_activation_script_does_not_choose_a_dds_implementation(tmp_path):
    foxy_setup = tmp_path / 'foxy_setup.bash'
    workspace_setup = tmp_path / 'workspace_setup.bash'
    sdk_prefix = tmp_path / 'librealsense'
    foxy_setup.write_text(
        'export FAKE_FOXY_SOURCED=yes\n'
        'export ROS_DISTRO=foxy\n',
        encoding='utf-8',
    )
    workspace_setup.write_text(
        'export FAKE_WORKSPACE_SOURCED=yes\n',
        encoding='utf-8',
    )

    environment = os.environ.copy()
    for variable in (
        'ROS_DISTRO',
        'RMW_IMPLEMENTATION',
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'REALSENSE_RMW_IMPLEMENTATION',
        'REALSENSE_FASTDDS_PROFILE',
    ):
        environment.pop(variable, None)
    environment.update({
        'REALSENSE_FOXY_SETUP': str(foxy_setup),
        'REALSENSE_256_PREFIX': str(sdk_prefix),
        'REALSENSE_456_SETUP': str(workspace_setup),
    })
    command = (
        f'source "{SCRIPT}"; '
        'printf "%s\\n" '
        '"$FAKE_FOXY_SOURCED" '
        '"$FAKE_WORKSPACE_SOURCED" '
        '"${RMW_IMPLEMENTATION-unset}" '
        '"${CYCLONEDDS_URI-unset}" '
        '"${FASTRTPS_DEFAULT_PROFILES_FILE-unset}"'
    )

    result = subprocess.run(
        ['/bin/bash', '--noprofile', '--norc', '-c', command],
        check=True,
        capture_output=True,
        env=environment,
        text=True,
    )

    assert result.stdout.splitlines() == [
        'yes',
        'yes',
        'unset',
        'unset',
        'unset',
    ]
