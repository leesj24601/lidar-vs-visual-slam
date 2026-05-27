#!/usr/bin/env python3
"""HTTP dashboard backend for Go2 RTAB-Map SLAM control."""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


DASHBOARD_DIR = Path(__file__).resolve().parent
REPO_ROOT = DASHBOARD_DIR.parent
ACTIVE_DB = REPO_ROOT / 'maps' / 'active' / 'rtabmap.db'
SESSIONS_DIR = REPO_ROOT / 'maps' / 'sessions'
LOG_DIR = Path('/tmp/go2_lidar_slam_dashboard')
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
WORKSPACE_SETUP = REPO_ROOT / 'install' / 'setup.bash'

SESSION_RE = re.compile(r'[^A-Za-z0-9_-]+')

DASHBOARD_ROS_PROCESS_MARKERS = (
    'go2_rtabmap_launch localization.launch.py',
    'go2_rtabmap_launch slam.launch.py',
    'go2_rtabmap_launch/share/go2_rtabmap_launch',
    'install/go2_rtabmap_bridge/lib/go2_rtabmap_bridge/bridge_node',
    '__node:=go2_rtabmap_bridge',
    '__node:=utlidar_static_tf',
)

ALIGN_CORRECTION_PARAMS = (
    ('RGBD/ProximityBySpace', "'true'"),
    ('RGBD/ProximityOdomGuess', "'false'"),
    ('RGBD/ProximityPathMaxNeighbors', "'1'"),
    ('RGBD/ProximityMaxGraphDepth', "'0'"),
    ('RGBD/ProximityGlobalScanMap', "'false'"),
    ('RGBD/LinearUpdate', "'0.05'"),
    ('RGBD/AngularUpdate', "'0.05'"),
    ('RGBD/MaxOdomCacheSize', "'10'"),
    ('RGBD/OptimizeMaxError', "'3.0'"),
    ('Icp/CorrespondenceRatio', "'0.2'"),
    ('Icp/MaxCorrespondenceDistance', "'1.0'"),
    ('Icp/OutlierRatio', "'0.7'"),
    ('Icp/MaxTranslation', "'3.0'"),
)

LOCK_CORRECTION_PARAMS = (
    ('RGBD/ProximityBySpace', "'false'"),
    ('RGBD/ProximityOdomGuess', "'false'"),
    ('RGBD/ProximityPathMaxNeighbors', "'0'"),
    ('RGBD/ProximityMaxGraphDepth', "'0'"),
    ('RGBD/ProximityGlobalScanMap', "'false'"),
    ('RGBD/LinearUpdate', "'0.1'"),
    ('RGBD/AngularUpdate', "'0.1'"),
    ('RGBD/MaxOdomCacheSize', "'10'"),
    ('RGBD/OptimizeMaxError', "'3.0'"),
    ('Icp/CorrespondenceRatio', "'0.2'"),
    ('Icp/MaxCorrespondenceDistance', "'1.0'"),
    ('Icp/OutlierRatio', "'0.7'"),
    ('Icp/MaxTranslation', "'0.25'"),
)


def sanitize_session_name(value: str) -> str:
    normalized = SESSION_RE.sub('_', value.strip())
    normalized = normalized.strip('_')
    return normalized or 'unnamed_session'


def correction_parameter_profile(enabled: bool):
    return ALIGN_CORRECTION_PARAMS if enabled else LOCK_CORRECTION_PARAMS


def is_dashboard_ros_process(command: str) -> bool:
    if 'dashboard/server.py' in command:
        return False
    return any(marker in command for marker in DASHBOARD_ROS_PROCESS_MARKERS)


def bool_value(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
    }


def parse_initial_pose(value: str):
    parts = [float(part) for part in value.split()]
    if len(parts) == 6:
        x, y, z, roll, pitch, yaw = parts
        quat = quaternion_from_euler(roll, pitch, yaw)
    elif len(parts) == 7:
        x, y, z, qx, qy, qz, qw = parts
        quat = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    else:
        raise ValueError(
            'initialPose must contain 6 values (x y z roll pitch yaw) '
            'or 7 values (x y z qx qy qz qw).'
        )

    return x, y, z, quat


class RosStatusNode(Node):
    def __init__(self):
        super().__init__('go2_dashboard_backend')
        self._lock = threading.Lock()
        self._proximity_id = None
        self._loop_closure_id = None
        self._pose_frame = None
        self._pose_times = []
        self._tf_last_seen_sec = None
        self._tf_seen_times = []
        self._tf_detail = 'no transform'
        self._tf_status = 'map->odom missing'

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/rtabmap/initialpose',
            10,
        )

        self.create_subscription(Info, '/rtabmap/info', self._info_callback, 10)
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self._pose_callback,
            10,
        )
        self.create_timer(0.5, self._tf_timer_callback)

    def _info_callback(self, msg: Info):
        with self._lock:
            self._proximity_id = getattr(msg, 'proximity_detection_id', None)
            self._loop_closure_id = getattr(msg, 'loop_closure_id', None)

    def _tf_callback(self, msg: TFMessage):
        now = time.monotonic()
        seen = any(
            transform.header.frame_id == 'map'
            and transform.child_frame_id == 'odom'
            for transform in msg.transforms
        )
        if not seen:
            return

        with self._lock:
            self._tf_last_seen_sec = now
            self._tf_seen_times.append(now)
            cutoff = now - 5.0
            self._tf_seen_times = [stamp for stamp in self._tf_seen_times if stamp >= cutoff]

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        now = time.monotonic()
        with self._lock:
            self._pose_frame = msg.header.frame_id or 'unknown'
            self._pose_times.append(now)
            cutoff = now - 10.0
            self._pose_times = [stamp for stamp in self._pose_times if stamp >= cutoff]

    def _tf_timer_callback(self):
        try:
            transform = self._tf_buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            with self._lock:
                if self._tf_last_seen_sec is None:
                    self._tf_status = 'map->odom missing'
                    self._tf_detail = 'no transform'
                else:
                    age = time.monotonic() - self._tf_last_seen_sec
                    self._tf_status = 'map->odom stale' if age <= 5.0 else 'map->odom missing'
                    self._tf_detail = f'last {age:.1f}s ago'
            return

        stamp = Time.from_msg(transform.header.stamp)
        now_ros = self.get_clock().now()
        age_sec = (now_ros - stamp).nanoseconds / 1_000_000_000.0
        if age_sec < 0.0:
            age_sec = 0.0
        now = time.monotonic()

        with self._lock:
            if self._tf_last_seen_sec is None:
                self._tf_status = 'map->odom missing'
                self._tf_detail = 'no transform'
                return

            receive_age = now - self._tf_last_seen_sec
            self._tf_status = 'map->odom live' if receive_age <= 1.0 else 'map->odom stale'
            self._tf_detail = f'last {receive_age:.1f}s ago'

    def publish_initial_pose(self, initial_pose: str):
        x, y, z, quat = parse_initial_pose(initial_pose)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat['x']
        msg.pose.pose.orientation.y = quat['y']
        msg.pose.pose.orientation.z = quat['z']
        msg.pose.pose.orientation.w = quat['w']
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        for _ in range(3):
            self._initial_pose_pub.publish(msg)
            time.sleep(0.05)

    def status(self):
        now = time.monotonic()
        with self._lock:
            pose_times = list(self._pose_times)
            pose_frame = self._pose_frame
            proximity_id = self._proximity_id
            loop_closure_id = self._loop_closure_id
            tf_status = self._tf_status
            tf_detail = self._tf_detail
            tf_times = list(self._tf_seen_times)

        pose_times = [stamp for stamp in pose_times if stamp >= now - 5.0]
        if pose_times:
            elapsed = max(now - pose_times[0], 1.0)
            rate = len(pose_times) / elapsed
            pose_stream = (
                '/rtabmap/localization_pose<br>'
                f'{pose_frame or "unknown"} frame · {rate:.1f} Hz'
            )
        else:
            pose_stream = '/rtabmap/localization_pose<br>waiting'

        tf_payload = {'status': tf_status, 'detail': tf_detail}
        if tf_detail.startswith('last ') and tf_detail.endswith('s ago'):
            try:
                tf_payload['lastSeenSec'] = float(tf_detail[5:-5])
            except ValueError:
                pass
        tf_times = [stamp for stamp in tf_times if stamp >= now - 5.0]
        if len(tf_times) >= 2:
            elapsed = max(tf_times[-1] - tf_times[0], 1.0)
            tf_payload['rateHz'] = len(tf_times) / elapsed

        return {
            'rtabmap': {
                'proximityId': proximity_id if proximity_id is not None else '--',
                'loopClosureId': loop_closure_id if loop_closure_id is not None else '--',
            },
            'tf': tf_payload,
            'poseStream': pose_stream,
        }


class DashboardBackend:
    def __init__(self, ros_node: RosStatusNode):
        self.ros_node = ros_node
        self._lock = threading.Lock()
        self.mapping_proc = None
        self.localization_proc = None
        self.correction_enabled = True
        LOG_DIR.mkdir(parents=True, exist_ok=True)

    def status(self):
        with self._lock:
            mapping_running = self._is_running(self.mapping_proc)
            localization_running = self._is_running(self.localization_proc)
            if not mapping_running:
                self.mapping_proc = None
            if not localization_running:
                self.localization_proc = None
            correction_enabled = self.correction_enabled

        status = self.ros_node.status()
        status.update({
            'mappingRunning': mapping_running,
            'localizationRunning': localization_running,
            'correctionEnabled': correction_enabled,
        })
        return status

    def start_mapping(self, payload):
        session_name = sanitize_session_name(str(payload.get('sessionName', '')))
        database_path = SESSIONS_DIR / session_name / 'rtabmap.db'
        sidecars = [
            database_path,
            Path(str(database_path) + '-wal'),
            Path(str(database_path) + '-shm'),
            Path(str(database_path) + '-journal'),
        ]

        with self._lock:
            if self._is_running(self.localization_proc):
                raise ApiError(409, 'Stop localization before starting mapping.')
            if self._is_running(self.mapping_proc):
                return self.status()
            if any(path.exists() for path in sidecars):
                raise ApiError(
                    409,
                    f'Mapping session already exists: maps/sessions/{session_name}',
                )

            database_path.parent.mkdir(parents=True, exist_ok=True)
            args = [
                'ros2',
                'launch',
                'go2_rtabmap_launch',
                'slam.launch.py',
                f'database_path:={database_path}',
                f'rtabmap_viz:={str(bool_value(payload.get("rtabmapViz", False))).lower()}',
            ]
            self.mapping_proc = self._start_process(args, 'mapping')

        return self.status()

    def stop_mapping(self):
        with self._lock:
            self._stop_process(self.mapping_proc)
            self.mapping_proc = None
        return self.status()

    def start_localization(self, payload):
        initial_pose = str(payload.get('initialPose', '')).strip()
        if not ACTIVE_DB.is_file():
            raise ApiError(404, f'Active localization DB does not exist: {ACTIVE_DB}')

        with self._lock:
            if self._is_running(self.mapping_proc):
                raise ApiError(409, 'Stop mapping before starting localization.')
            if self._is_running(self.localization_proc):
                if initial_pose:
                    self.ros_node.publish_initial_pose(initial_pose)
                return self.status()

            args = [
                'ros2',
                'launch',
                'go2_rtabmap_launch',
                'localization.launch.py',
                f'database_path:={ACTIVE_DB}',
                f'rtabmap_viz:={str(bool_value(payload.get("rtabmapViz", True))).lower()}',
            ]
            if initial_pose:
                parse_initial_pose(initial_pose)
                args.append(f'initial_pose:={initial_pose}')
            self.localization_proc = self._start_process(args, 'localization')
            self.correction_enabled = True

        return self.status()

    def stop_localization(self):
        with self._lock:
            self._stop_process(self.localization_proc)
            self.localization_proc = None
        return self.status()

    def send_pose(self, payload):
        initial_pose = str(payload.get('initialPose', '')).strip()
        if not initial_pose:
            raise ApiError(400, 'initialPose is required.')
        with self._lock:
            if not self._is_running(self.localization_proc):
                raise ApiError(409, 'Start localization before sending pose.')
        self.ros_node.publish_initial_pose(initial_pose)
        return self.status()

    def set_correction(self, enabled: bool):
        with self._lock:
            if not self._is_running(self.localization_proc):
                raise ApiError(409, 'Start localization before changing correction.')

        for param_name, value in correction_parameter_profile(enabled):
            self._run_ros2([
                'ros2',
                'param',
                'set',
                '/rtabmap/rtabmap',
                param_name,
                value,
            ])
        self._run_ros2([
            'ros2',
            'service',
            'call',
            '/rtabmap/rtabmap/update_parameters',
            'std_srvs/srv/Empty',
            '{}',
        ])
        with self._lock:
            self.correction_enabled = enabled
        return self.status()

    def kill_all(self):
        with self._lock:
            tracked = [self.mapping_proc, self.localization_proc]
            self.mapping_proc = None
            self.localization_proc = None
            self.correction_enabled = False

        for proc in tracked:
            self._stop_process(proc)

        killed_groups = []
        for pgid in self._matching_process_groups():
            if self._stop_process_group(pgid):
                killed_groups.append(pgid)

        time.sleep(0.5)
        status = self.status()
        status['killedProcessGroups'] = killed_groups
        return status

    def shutdown(self):
        with self._lock:
            self._stop_process(self.mapping_proc)
            self._stop_process(self.localization_proc)
            self.mapping_proc = None
            self.localization_proc = None

    @staticmethod
    def _is_running(proc):
        return proc is not None and proc.poll() is None

    @staticmethod
    def _start_process(args, name):
        log_path = LOG_DIR / f'{name}.log'
        log_file = log_path.open('ab')
        proc = subprocess.Popen(
            DashboardBackend._with_ros_environment(args),
            cwd=str(REPO_ROOT),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        time.sleep(0.5)
        if proc.poll() is not None:
            raise ApiError(
                500,
                f'{name} launch exited immediately. {DashboardBackend._tail_log(log_path)}',
            )
        return proc

    @staticmethod
    def _stop_process(proc):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    @staticmethod
    def _matching_process_groups():
        completed = subprocess.run(
            ['ps', '-eo', 'pid=,pgid=,cmd='],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        groups = set()
        own_pid = os.getpid()
        own_pgid = os.getpgrp()
        for line in completed.stdout.splitlines():
            parts = line.strip().split(None, 2)
            if len(parts) != 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            command = parts[2]
            if pid == own_pid or pgid == own_pgid:
                continue
            if is_dashboard_ros_process(command):
                groups.add(pgid)
        return sorted(groups)

    @staticmethod
    def _stop_process_group(pgid: int) -> bool:
        if pgid <= 1:
            return False
        if not DashboardBackend._process_group_exists(pgid):
            return False
        for sig, delay in (
            (signal.SIGINT, 2.0),
            (signal.SIGTERM, 1.0),
            (signal.SIGKILL, 0.0),
        ):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return True
            if delay:
                time.sleep(delay)
            if not DashboardBackend._process_group_exists(pgid):
                return True
        return not DashboardBackend._process_group_exists(pgid)

    @staticmethod
    def _process_group_exists(pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    @staticmethod
    def _run_ros2(args):
        try:
            completed = subprocess.run(
                DashboardBackend._with_ros_environment(args),
                cwd=str(REPO_ROOT),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=8,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise ApiError(504, f'Command timed out: {" ".join(args)}') from exc
        if completed.returncode != 0:
            output = completed.stdout.strip() or 'ROS2 command failed.'
            status = 503 if '/rtabmap/rtabmap' in ' '.join(args) else 500
            raise ApiError(status, output)

    @staticmethod
    def _with_ros_environment(args):
        setup_commands = []
        if ROS_SETUP.is_file():
            setup_commands.append(f'source {shlex.quote(str(ROS_SETUP))}')
        if WORKSPACE_SETUP.is_file():
            setup_commands.append(f'source {shlex.quote(str(WORKSPACE_SETUP))}')
        quoted_args = ' '.join(shlex.quote(str(arg)) for arg in args)
        command = ' && '.join([*setup_commands, f'exec {quoted_args}'])
        return ['bash', '-lc', command]

    @staticmethod
    def _tail_log(path: Path):
        if not path.is_file():
            return ''
        lines = path.read_text(errors='replace').splitlines()
        tail = '\n'.join(lines[-8:])
        return tail.strip()


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


def make_handler(backend: DashboardBackend):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'Go2Dashboard/0.1'

        def log_message(self, fmt, *args):
            sys.stderr.write('%s - - [%s] %s\n' % (
                self.address_string(),
                self.log_date_time_string(),
                fmt % args,
            ))

        def end_headers(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            super().end_headers()

        def do_OPTIONS(self):
            self.send_response(204)
            self.end_headers()

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/api/status':
                self._json_response(200, backend.status())
                return
            if parsed.path == '/' and not parsed.query:
                self.send_response(302)
                self.send_header('Location', '/?api=1')
                self.end_headers()
                return
            self._serve_static(parsed.path)

        def do_POST(self):
            parsed = urlparse(self.path)
            try:
                payload = self._read_json()
                if parsed.path == '/api/mapping/start':
                    data = backend.start_mapping(payload)
                elif parsed.path == '/api/mapping/stop':
                    data = backend.stop_mapping()
                elif parsed.path == '/api/localization/start':
                    data = backend.start_localization(payload)
                elif parsed.path == '/api/localization/pose':
                    data = backend.send_pose(payload)
                elif parsed.path == '/api/localization/stop':
                    data = backend.stop_localization()
                elif parsed.path == '/api/correction/align':
                    data = backend.set_correction(True)
                elif parsed.path == '/api/correction/lock':
                    data = backend.set_correction(False)
                elif parsed.path == '/api/system/kill-all':
                    data = backend.kill_all()
                else:
                    raise ApiError(404, f'Unknown endpoint: {parsed.path}')
                self._json_response(200, data)
            except ApiError as exc:
                self._json_response(exc.status, {'error': exc.message})
            except ValueError as exc:
                self._json_response(400, {'error': str(exc)})

        def _read_json(self):
            length = int(self.headers.get('Content-Length', '0'))
            if length == 0:
                return {}
            if length > 64 * 1024:
                raise ApiError(413, 'Request body too large.')
            raw = self.rfile.read(length)
            return json.loads(raw.decode('utf-8'))

        def _json_response(self, status, data):
            body = json.dumps(data, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_static(self, request_path):
            path = request_path.lstrip('/') or 'index.html'
            if path == '':
                path = 'index.html'
            file_path = (DASHBOARD_DIR / path).resolve()
            if DASHBOARD_DIR not in file_path.parents and file_path != DASHBOARD_DIR:
                self.send_error(403)
                return
            if file_path.is_dir():
                file_path = file_path / 'index.html'
            if not file_path.is_file():
                self.send_error(404)
                return

            content_type = mimetypes.guess_type(str(file_path))[0] or 'application/octet-stream'
            body = file_path.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def spin_executor(executor: SingleThreadedExecutor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    parser = argparse.ArgumentParser(description='Go2 RTAB-Map dashboard backend')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()

    rclpy.init()
    ros_node = RosStatusNode()
    backend = DashboardBackend(ros_node)
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=spin_executor, args=(executor,), daemon=True)
    spin_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(backend))
    print(f'Serving Go2 dashboard on http://{args.host}:{args.port}/?api=1')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        backend.shutdown()
        server.server_close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        ros_node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
