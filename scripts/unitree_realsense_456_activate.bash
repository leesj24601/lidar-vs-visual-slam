#!/usr/bin/env bash

REALSENSE_FOXY_SETUP="${REALSENSE_FOXY_SETUP:-/opt/ros/foxy/setup.bash}"
REALSENSE_256_PREFIX="${REALSENSE_256_PREFIX:-/home/unitree/librealsense-2.56.5-install}"
REALSENSE_456_SETUP="${REALSENSE_456_SETUP:-/home/unitree/ros2_realsense_456_ws/install/local_setup.bash}"

if [[ "${ROS_DISTRO:-}" != "foxy" ]]; then
  source "${REALSENSE_FOXY_SETUP}"
fi

export PATH="${REALSENSE_256_PREFIX}/bin${PATH:+:${PATH}}"
export CMAKE_PREFIX_PATH="${REALSENSE_256_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${REALSENSE_256_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

source "${REALSENSE_456_SETUP}"
