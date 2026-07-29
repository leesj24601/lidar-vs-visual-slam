import math
from collections import deque

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster


_NANOSECONDS_PER_SECOND = 1_000_000_000


def _stamp_nanoseconds(msg):
    stamp = msg.header.stamp
    return stamp.sec * _NANOSECONDS_PER_SECOND + stamp.nanosec


def _normalized_quaternion(quaternion):
    values = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if not all(math.isfinite(value) for value in values):
        return None

    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        return None
    return tuple(value / norm for value in values)


def _yaw_from_quaternion(quaternion):
    normalized = _normalized_quaternion(quaternion)
    if normalized is None:
        raise ValueError('odometry pose has an invalid quaternion')

    x, y, z, w = normalized
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _is_valid_odometry(msg):
    position = msg.pose.pose.position
    return (
        bool(msg.header.frame_id.strip())
        and math.isfinite(position.x)
        and math.isfinite(position.y)
        and _normalized_quaternion(msg.pose.pose.orientation) is not None
    )


def inverse_planar_origin_transform(
    first_odom,
    parent_frame_id,
    transform_stamp,
):
    """Return the planar inverse of an odometry's first pose."""
    if not _is_valid_odometry(first_odom):
        raise ValueError('first odometry pose is invalid')
    if not parent_frame_id.strip():
        raise ValueError('parent frame id must not be empty')

    position = first_odom.pose.pose.position
    yaw = _yaw_from_quaternion(first_odom.pose.pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    inverse_yaw = -yaw

    transform = TransformStamped()
    transform.header.stamp = transform_stamp
    transform.header.frame_id = parent_frame_id
    transform.child_frame_id = first_odom.header.frame_id
    transform.transform.translation.x = (
        -cosine * position.x - sine * position.y
    )
    transform.transform.translation.y = (
        sine * position.x - cosine * position.y
    )
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = math.sin(inverse_yaw * 0.5)
    transform.transform.rotation.w = math.cos(inverse_yaw * 0.5)
    return transform


def find_nearest_valid_pair(
    go2_messages,
    vo_messages,
    max_time_gap_ns,
):
    """Find the valid Go2/VO pair with the smallest timestamp difference."""
    if max_time_gap_ns < 0:
        raise ValueError('maximum timestamp gap must not be negative')

    best_pair = None
    best_gap = None
    for go2_msg in go2_messages:
        if not _is_valid_odometry(go2_msg):
            continue
        go2_stamp = _stamp_nanoseconds(go2_msg)
        for vo_msg in vo_messages:
            if not _is_valid_odometry(vo_msg):
                continue
            gap = abs(go2_stamp - _stamp_nanoseconds(vo_msg))
            if gap <= max_time_gap_ns and (
                best_gap is None or gap < best_gap
            ):
                best_pair = (go2_msg, vo_msg)
                best_gap = gap
    return best_pair


class OdomInitialAlignmentTf(Node):
    """Align the first synchronized Go2 and VO planar poses for RViz."""

    def __init__(self):
        super().__init__('odom_initial_alignment_tf')

        self.declare_parameter('go2_odom_topic', '/odom/go2')
        self.declare_parameter('vo_odom_topic', '/odom/vo')
        self.declare_parameter('comparison_frame_id', 'odom_compare')
        self.declare_parameter('max_time_gap_sec', 0.05)
        self.declare_parameter('buffer_size', 200)

        self._go2_odom_topic = self._string_param('go2_odom_topic')
        self._vo_odom_topic = self._string_param('vo_odom_topic')
        self._comparison_frame_id = self._string_param(
            'comparison_frame_id'
        )
        max_time_gap_sec = self._double_param('max_time_gap_sec')
        buffer_size = self._int_param('buffer_size')

        if not self._comparison_frame_id:
            raise ValueError('comparison_frame_id must not be empty')
        if max_time_gap_sec < 0.0:
            raise ValueError('max_time_gap_sec must not be negative')
        if buffer_size <= 0:
            raise ValueError('buffer_size must be positive')

        self._max_time_gap_ns = round(
            max_time_gap_sec * _NANOSECONDS_PER_SECOND
        )
        self._go2_messages = deque(maxlen=buffer_size)
        self._vo_messages = deque(maxlen=buffer_size)
        self._aligned = False
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        odom_qos = QoSProfile(
            depth=min(buffer_size, 100),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._go2_subscription = self.create_subscription(
            Odometry,
            self._go2_odom_topic,
            self._go2_callback,
            odom_qos,
        )
        self._vo_subscription = self.create_subscription(
            Odometry,
            self._vo_odom_topic,
            self._vo_callback,
            odom_qos,
        )

        self.get_logger().info(
            'Waiting to align first synchronized odometry poses: '
            f'{self._go2_odom_topic}, {self._vo_odom_topic} '
            f'-> {self._comparison_frame_id} '
            f'(max gap {max_time_gap_sec:.3f} s)'
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _go2_callback(self, msg):
        if self._aligned:
            return
        self._go2_messages.append(msg)
        self._try_align()

    def _vo_callback(self, msg):
        if self._aligned:
            return
        self._vo_messages.append(msg)
        self._try_align()

    def _try_align(self):
        pair = find_nearest_valid_pair(
            self._go2_messages,
            self._vo_messages,
            self._max_time_gap_ns,
        )
        if pair is None:
            return

        go2_msg, vo_msg = pair
        child_frames = {
            go2_msg.header.frame_id,
            vo_msg.header.frame_id,
        }
        if len(child_frames) != 2:
            self.get_logger().error(
                'Cannot align odometries with the same frame id: '
                f'{go2_msg.header.frame_id}'
            )
            return
        if self._comparison_frame_id in child_frames:
            self.get_logger().error(
                'comparison_frame_id must differ from odometry frame ids'
            )
            return

        transform_stamp = self.get_clock().now().to_msg()
        transforms = [
            inverse_planar_origin_transform(
                go2_msg,
                self._comparison_frame_id,
                transform_stamp,
            ),
            inverse_planar_origin_transform(
                vo_msg,
                self._comparison_frame_id,
                transform_stamp,
            ),
        ]
        self._tf_broadcaster.sendTransform(transforms)
        self._aligned = True

        gap_ms = abs(
            _stamp_nanoseconds(go2_msg) - _stamp_nanoseconds(vo_msg)
        ) / 1_000_000.0
        self.get_logger().info(
            'Published initial planar alignment TFs: '
            f'{self._comparison_frame_id} -> '
            f'{go2_msg.header.frame_id}, {vo_msg.header.frame_id}; '
            f'first-pose timestamp gap={gap_ms:.3f} ms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomInitialAlignmentTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
