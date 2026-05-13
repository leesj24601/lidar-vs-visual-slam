import copy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_sensor_msgs import transform_points


class Go2RtabmapBridge(Node):
    """Normalize Go2 LiDAR odometry and point clouds for RTAB-Map."""

    def __init__(self):
        super().__init__('go2_rtabmap_bridge')

        self.declare_parameter('input_odom_topic', '/utlidar/robot_odom')
        self.declare_parameter('input_cloud_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('output_cloud_topic', '/scan_cloud')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_qos_depth', 50)
        self.declare_parameter('cloud_qos_depth', 10)
        self.declare_parameter('tf_cache_time_sec', 10.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.0)
        self.declare_parameter('tf_latest_fallback_tolerance_sec', 0.2)

        self._input_odom_topic = self._string_param('input_odom_topic')
        self._input_cloud_topic = self._string_param('input_cloud_topic')
        self._output_odom_topic = self._string_param('output_odom_topic')
        self._output_cloud_topic = self._string_param('output_cloud_topic')
        self._odom_frame_id = self._string_param('odom_frame_id')
        self._base_frame_id = self._string_param('base_frame_id')
        self._tf_lookup_timeout = Duration(
            seconds=self._double_param('tf_lookup_timeout_sec')
        )
        self._tf_latest_fallback_tolerance = Duration(
            seconds=self._double_param('tf_latest_fallback_tolerance_sec')
        )

        tf_cache_time = Duration(seconds=self._double_param('tf_cache_time_sec'))
        self._tf_buffer = Buffer(cache_time=tf_cache_time, node=self)
        self._tf_broadcaster = TransformBroadcaster(self)

        odom_qos = QoSProfile(
            depth=self._int_param('odom_qos_depth'),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        cloud_qos = QoSProfile(
            depth=self._int_param('cloud_qos_depth'),
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        self._cloud_pub = self.create_publisher(
            PointCloud2, self._output_cloud_topic, output_qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, self._input_odom_topic, self._odom_callback, odom_qos
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, self._input_cloud_topic, self._cloud_callback, cloud_qos
        )

        self._time_offset = None
        self._cloud_drops_before_offset = 0
        self._cloud_drops_tf = 0

        self.get_logger().info(
            'Go2 RTAB-Map bridge started: '
            f'{self._input_odom_topic} -> {self._output_odom_topic}, '
            f'{self._input_cloud_topic} -> {self._output_cloud_topic}'
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _odom_callback(self, msg):
        corrected_stamp = self._correct_odom_stamp(msg.header.stamp)

        transform = self._transform_from_odom(msg, corrected_stamp)
        self._tf_broadcaster.sendTransform(transform)
        self._tf_buffer.set_transform(transform, self.get_name())

        odom_out = copy.deepcopy(msg)
        odom_out.header.stamp = corrected_stamp
        odom_out.header.frame_id = self._odom_frame_id
        odom_out.child_frame_id = self._base_frame_id
        self._odom_pub.publish(odom_out)

    def _cloud_callback(self, msg):
        if self._time_offset is None:
            self._cloud_drops_before_offset += 1
            if self._cloud_drops_before_offset <= 5:
                self.get_logger().warn(
                    'Dropping cloud before first odom timestamp offset is available.'
                )
            return

        if not msg.header.frame_id:
            self._cloud_drops_tf += 1
            self._warn_cloud_drop('cloud has empty header.frame_id')
            return

        corrected_time = self._corrected_time(msg.header.stamp)
        corrected_stamp = corrected_time.to_msg()

        try:
            transform = self._lookup_cloud_transform(msg.header.frame_id, corrected_time)
        except TransformException as exc:
            self._cloud_drops_tf += 1
            self._warn_cloud_drop(str(exc))
            return

        cloud_out = self._transform_cloud_preserve_layout(msg, transform)
        cloud_out.header.stamp = corrected_stamp
        cloud_out.header.frame_id = self._base_frame_id
        self._cloud_pub.publish(cloud_out)

    def _correct_odom_stamp(self, stamp):
        sensor_time = Time.from_msg(stamp)
        if self._time_offset is None:
            self._time_offset = self.get_clock().now() - sensor_time
            offset_sec = self._time_offset.nanoseconds / 1_000_000_000.0
            self.get_logger().info(
                f'Initialized Go2 timestamp offset from odom: {offset_sec:.6f} sec'
            )
        return (sensor_time + self._time_offset).to_msg()

    def _corrected_time(self, stamp):
        return Time.from_msg(stamp) + self._time_offset

    def _transform_from_odom(self, msg, corrected_stamp):
        transform = TransformStamped()
        transform.header.stamp = corrected_stamp
        transform.header.frame_id = self._odom_frame_id
        transform.child_frame_id = self._base_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        return transform

    def _warn_cloud_drop(self, reason):
        if self._cloud_drops_tf <= 5 or self._cloud_drops_tf % 100 == 0:
            self.get_logger().warn(
                f'Dropping cloud; transform to {self._base_frame_id} unavailable: '
                f'{reason}'
            )

    def _transform_cloud_preserve_layout(self, cloud, transform_stamped):
        fields = {field.name: field for field in cloud.fields}
        missing_fields = {'x', 'y', 'z'} - set(fields)
        if missing_fields:
            raise ValueError(f'PointCloud2 missing fields: {sorted(missing_fields)}')

        for field_name in ('x', 'y', 'z'):
            field = fields[field_name]
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise ValueError(
                    f'PointCloud2 field {field_name} must be FLOAT32 count=1, '
                    f'got datatype={field.datatype} count={field.count}'
                )

        cloud_out = copy.deepcopy(cloud)
        data = bytearray(cloud.data)
        dtype = np.dtype('>f4' if cloud.is_bigendian else '<f4')
        height = cloud.height or 1
        width = cloud.width
        shape = (height, width)

        arrays = []
        for field_name in ('x', 'y', 'z'):
            arrays.append(
                np.ndarray(
                    shape=shape,
                    dtype=dtype,
                    buffer=data,
                    offset=fields[field_name].offset,
                    strides=(cloud.row_step, cloud.point_step),
                )
            )

        xyz = np.stack([array.reshape(-1) for array in arrays], axis=1)
        transformed_xyz = transform_points(xyz, transform_stamped.transform)
        for index, array in enumerate(arrays):
            array[:, :] = transformed_xyz[:, index].reshape(shape)

        cloud_out.data = data
        return cloud_out

    def _lookup_cloud_transform(self, source_frame, corrected_time):
        try:
            return self._tf_buffer.lookup_transform(
                self._base_frame_id,
                source_frame,
                corrected_time,
                timeout=self._tf_lookup_timeout,
            )
        except TransformException as exact_lookup_error:
            latest_transform = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
            latest_time = Time.from_msg(latest_transform.header.stamp)
            delta = corrected_time - latest_time
            if abs(delta.nanoseconds) <= self._tf_latest_fallback_tolerance.nanoseconds:
                return latest_transform
            raise exact_lookup_error


def main(args=None):
    rclpy.init(args=args)
    node = Go2RtabmapBridge()
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
