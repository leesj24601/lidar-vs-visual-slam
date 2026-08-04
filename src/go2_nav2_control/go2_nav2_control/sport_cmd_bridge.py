import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

from go2_nav2_control.command_policy import SportCommandPolicy


DEFAULT_MIN_LINEAR_X = -0.50
DEFAULT_MAX_LINEAR_X = 1.00
DEFAULT_MAX_LINEAR_Y = 0.40
DEFAULT_MAX_ANGULAR_Z = 1.00


def build_request(command):
    request = Request()
    request.header.identity.api_id = command.api_id
    request.parameter = command.parameter
    return request


class SportCmdBridge(Node):
    def __init__(self):
        super().__init__('go2_sport_cmd_bridge')
        self.declare_parameter('enabled', False)
        self.declare_parameter('min_linear_x', DEFAULT_MIN_LINEAR_X)
        self.declare_parameter('max_linear_x', DEFAULT_MAX_LINEAR_X)
        self.declare_parameter('max_linear_y', DEFAULT_MAX_LINEAR_Y)
        self.declare_parameter('max_angular_z', DEFAULT_MAX_ANGULAR_Z)
        self.declare_parameter('cmd_vel_timeout', 0.30)
        self.declare_parameter('watchdog_period', 0.05)

        watchdog_period = self.get_parameter(
            'watchdog_period'
        ).get_parameter_value().double_value
        if watchdog_period <= 0.0:
            raise ValueError('watchdog_period must be positive')

        self._policy = SportCommandPolicy(
            min_linear_x=self.get_parameter(
                'min_linear_x'
            ).get_parameter_value().double_value,
            max_linear_x=self.get_parameter(
                'max_linear_x'
            ).get_parameter_value().double_value,
            max_linear_y=self.get_parameter(
                'max_linear_y'
            ).get_parameter_value().double_value,
            max_angular_z=self.get_parameter(
                'max_angular_z'
            ).get_parameter_value().double_value,
            cmd_vel_timeout=self.get_parameter(
                'cmd_vel_timeout'
            ).get_parameter_value().double_value,
            enabled=self.get_parameter(
                'enabled'
            ).get_parameter_value().bool_value,
        )

        self._request_publisher = self.create_publisher(
            Request,
            '/api/sport/request',
            10,
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(
            watchdog_period,
            self._watchdog_callback,
        )
        state = 'enabled' if self._policy.enabled else 'disabled'
        self.get_logger().info(
            f'Go2 Sport command bridge is {state}; input=cmd_vel, '
            'output=/api/sport/request'
        )

    def _publish(self, command):
        if command is not None:
            self._request_publisher.publish(build_request(command))

    def _cmd_vel_callback(self, message):
        command = self._policy.accept_velocity(
            vx=message.linear.x,
            vy=message.linear.y,
            vyaw=message.angular.z,
            now_sec=time.monotonic(),
        )
        self._publish(command)

    def _watchdog_callback(self):
        self._publish(self._policy.check_timeout(time.monotonic()))

    def publish_shutdown_stop(self):
        self._publish(self._policy.shutdown_command())


def main(args=None):
    rclpy.init(args=args)
    node = SportCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_shutdown_stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
