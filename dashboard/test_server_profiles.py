#!/usr/bin/env python3
"""Unit tests for dashboard backend RTAB-Map parameter profiles."""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

from server import correction_parameter_profile, is_dashboard_ros_process  # noqa: E402


class CorrectionParameterProfileTest(unittest.TestCase):
    def test_align_profile_uses_official_lidar_localization_shape(self):
        profile = dict(correction_parameter_profile(True))

        self.assertEqual(profile['RGBD/ProximityBySpace'], "'true'")
        self.assertEqual(profile['RGBD/ProximityOdomGuess'], "'false'")
        self.assertEqual(profile['RGBD/ProximityPathMaxNeighbors'], "'1'")
        self.assertEqual(profile['RGBD/ProximityMaxGraphDepth'], "'0'")
        self.assertEqual(profile['RGBD/ProximityGlobalScanMap'], "'false'")
        self.assertEqual(profile['RGBD/LinearUpdate'], "'0.05'")
        self.assertEqual(profile['RGBD/AngularUpdate'], "'0.05'")
        self.assertEqual(profile['RGBD/MaxOdomCacheSize'], "'10'")
        self.assertEqual(profile['RGBD/OptimizeMaxError'], "'3.0'")
        self.assertEqual(profile['Icp/CorrespondenceRatio'], "'0.2'")
        self.assertEqual(profile['Icp/MaxCorrespondenceDistance'], "'1.0'")
        self.assertEqual(profile['Icp/OutlierRatio'], "'0.7'")
        self.assertEqual(profile['Icp/MaxTranslation'], "'3.0'")

    def test_lock_profile_restores_odom_only_tracking(self):
        profile = dict(correction_parameter_profile(False))

        self.assertEqual(profile['RGBD/ProximityBySpace'], "'false'")
        self.assertEqual(profile['RGBD/ProximityOdomGuess'], "'false'")
        self.assertEqual(profile['RGBD/ProximityPathMaxNeighbors'], "'0'")
        self.assertEqual(profile['RGBD/ProximityMaxGraphDepth'], "'0'")
        self.assertEqual(profile['RGBD/ProximityGlobalScanMap'], "'false'")
        self.assertEqual(profile['RGBD/LinearUpdate'], "'0.1'")
        self.assertEqual(profile['RGBD/AngularUpdate'], "'0.1'")
        self.assertEqual(profile['RGBD/MaxOdomCacheSize'], "'10'")
        self.assertEqual(profile['Icp/CorrespondenceRatio'], "'0.2'")
        self.assertEqual(profile['Icp/MaxCorrespondenceDistance'], "'1.0'")
        self.assertEqual(profile['Icp/OutlierRatio'], "'0.7'")
        self.assertEqual(profile['Icp/MaxTranslation'], "'0.25'")


class KillAllProcessMatchingTest(unittest.TestCase):
    def test_matches_dashboard_owned_ros_processes(self):
        commands = [
            '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch go2_rtabmap_launch localization.launch.py',
            '/opt/ros/humble/lib/rtabmap_slam/rtabmap --params-file /home/cvr/Desktop/sj/go2_lidar_slam/install/go2_rtabmap_launch/share/go2_rtabmap_launch/config/rtabmap_lidar_indoor.yaml',
            '/usr/bin/python3 /home/cvr/Desktop/sj/go2_lidar_slam/install/go2_rtabmap_bridge/lib/go2_rtabmap_bridge/bridge_node',
            '/opt/ros/humble/lib/tf2_ros/static_transform_publisher --ros-args -r __node:=utlidar_static_tf',
        ]

        for command in commands:
            self.assertTrue(is_dashboard_ros_process(command), command)

    def test_does_not_match_dashboard_server_or_unrelated_ros_processes(self):
        commands = [
            'python3 dashboard/server.py --host 127.0.0.1 --port 8080',
            '/opt/ros/humble/bin/ros2 topic echo /rtabmap/info',
            '/opt/ros/humble/lib/rviz2/rviz2',
        ]

        for command in commands:
            self.assertFalse(is_dashboard_ros_process(command), command)


if __name__ == '__main__':
    unittest.main()
