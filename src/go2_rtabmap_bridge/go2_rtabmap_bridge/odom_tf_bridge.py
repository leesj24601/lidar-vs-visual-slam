import copy
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


def apply_time_offset(stamp, offset):
    return (Time.from_msg(stamp) + offset).to_msg()


def corrected_odom_stamp(stamp, clock_epoch_offset, sensor_time_offset_sec):
    """Map Go2 clock epoch and then apply the measured sensor residual."""
    sensor_offset_ns = round(sensor_time_offset_sec * 1_000_000_000.0)
    total_offset = Duration(
        nanoseconds=clock_epoch_offset.nanoseconds + sensor_offset_ns
    )
    return apply_time_offset(stamp, total_offset)


def yaw_from_quaternion(quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    siny_cosp = 2.0 * (
        w * z + x * y
    )
    cosy_cosp = 1.0 - 2.0 * (
        y * y + z * z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalized_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def inverse_quaternion(quaternion):
    x, y, z, w = quaternion
    return -x, -y, -z, w


def yaw_quaternion(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def set_quaternion(target, quaternion):
    target.x, target.y, target.z, target.w = quaternion


def apply_planar_motion(msg, footprint_frame_id):
    yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    planar_quat = yaw_quaternion(yaw)

    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = planar_quat[2]
    msg.pose.pose.orientation.w = planar_quat[3]
    msg.child_frame_id = footprint_frame_id


def apply_planar_base_motion(msg, base_frame_id):
    apply_planar_motion(msg, base_frame_id)


def transform_from_odom(
    msg,
    corrected_stamp,
    odom_frame_id,
    base_frame_id,
):
    transform = TransformStamped()
    transform.header.stamp = corrected_stamp
    transform.header.frame_id = odom_frame_id
    transform.child_frame_id = base_frame_id
    transform.transform.translation.x = msg.pose.pose.position.x
    transform.transform.translation.y = msg.pose.pose.position.y
    transform.transform.translation.z = msg.pose.pose.position.z
    transform.transform.rotation = msg.pose.pose.orientation
    return transform


def split_planar_and_body_transforms(
    msg,
    corrected_stamp,
    odom_frame_id,
    footprint_frame_id,
    base_frame_id,
):
    yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    planar_quat = yaw_quaternion(yaw)
    original_quat = normalized_quaternion(msg.pose.pose.orientation)
    body_quat = multiply_quaternions(
        inverse_quaternion(planar_quat),
        original_quat,
    )

    odom_to_footprint = TransformStamped()
    odom_to_footprint.header.stamp = corrected_stamp
    odom_to_footprint.header.frame_id = odom_frame_id
    odom_to_footprint.child_frame_id = footprint_frame_id
    odom_to_footprint.transform.translation.x = msg.pose.pose.position.x
    odom_to_footprint.transform.translation.y = msg.pose.pose.position.y
    odom_to_footprint.transform.translation.z = 0.0
    set_quaternion(odom_to_footprint.transform.rotation, planar_quat)

    footprint_to_base = TransformStamped()
    footprint_to_base.header.stamp = corrected_stamp
    footprint_to_base.header.frame_id = footprint_frame_id
    footprint_to_base.child_frame_id = base_frame_id
    footprint_to_base.transform.translation.x = 0.0
    footprint_to_base.transform.translation.y = 0.0
    footprint_to_base.transform.translation.z = msg.pose.pose.position.z
    set_quaternion(footprint_to_base.transform.rotation, body_quat)

    return [odom_to_footprint, footprint_to_base]


class Go2OdomTfBridge(Node):
    """Normalize Go2 odometry timestamps and optionally publish odom TF."""

    def __init__(self):
        super().__init__('go2_odom_tf_bridge')

        self.declare_parameter('input_odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('footprint_frame_id', '')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_qos_depth', 50)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('planarize_base_frame', False)
        self.declare_parameter('sensor_time_offset_sec', 0.0)

        self._input_odom_topic = self._string_param('input_odom_topic')
        self._output_odom_topic = self._string_param('output_odom_topic')
        self._odom_frame_id = self._string_param('odom_frame_id')
        self._footprint_frame_id = self._string_param('footprint_frame_id')
        self._base_frame_id = self._string_param('base_frame_id')
        self._publish_tf = self._bool_param('publish_tf')
        self._planarize_base_frame = self._bool_param('planarize_base_frame')
        self._sensor_time_offset_sec = self._double_param(
            'sensor_time_offset_sec'
        )

        odom_qos = QoSProfile(
            depth=self._int_param('odom_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._odom_pub = self.create_publisher(
            Odometry, self._output_odom_topic, output_qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._input_odom_topic, self._odom_callback, odom_qos
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._time_offset = None

        tf_state = 'enabled' if self._publish_tf else 'disabled'
        self.get_logger().info(
            'Go2 odom TF bridge started: '
            f'{self._input_odom_topic} -> {self._output_odom_topic}, '
            f'{self._odom_frame_id} -> {self._base_frame_id} TF {tf_state}'
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _bool_param(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _odom_callback(self, msg):
        corrected_stamp = self._correct_odom_stamp(msg.header.stamp)

        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(
                self._transforms_from_odom(msg, corrected_stamp)
            )

        odom_out = copy.deepcopy(msg)
        odom_out.header.stamp = corrected_stamp
        odom_out.header.frame_id = self._odom_frame_id
        if self._planarize_base_frame:
            apply_planar_base_motion(odom_out, self._base_frame_id)
        elif self._footprint_frame_id:
            apply_planar_motion(odom_out, self._footprint_frame_id)
        else:
            odom_out.child_frame_id = self._base_frame_id
        self._odom_pub.publish(odom_out)

    def _transforms_from_odom(self, msg, corrected_stamp):
        if self._planarize_base_frame:
            odom_out = copy.deepcopy(msg)
            apply_planar_base_motion(odom_out, self._base_frame_id)
            return transform_from_odom(
                odom_out,
                corrected_stamp,
                self._odom_frame_id,
                self._base_frame_id,
            )
        if self._footprint_frame_id:
            return split_planar_and_body_transforms(
                msg,
                corrected_stamp,
                self._odom_frame_id,
                self._footprint_frame_id,
                self._base_frame_id,
            )
        return transform_from_odom(
            msg,
            corrected_stamp,
            self._odom_frame_id,
            self._base_frame_id,
        )

    def _correct_odom_stamp(self, stamp):
        sensor_time = Time.from_msg(stamp)
        if self._time_offset is None:
            self._time_offset = self.get_clock().now() - sensor_time
            offset_sec = self._time_offset.nanoseconds / 1_000_000_000.0
            self.get_logger().info(
                f'Initialized Go2 odom timestamp offset: {offset_sec:.6f} sec'
            )
        return corrected_odom_stamp(
            stamp,
            self._time_offset,
            self._sensor_time_offset_sec,
        )


def main(args=None):
    rclpy.init(args=args)
    node = Go2OdomTfBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
