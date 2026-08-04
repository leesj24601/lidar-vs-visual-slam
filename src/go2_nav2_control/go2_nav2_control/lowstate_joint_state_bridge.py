import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

from go2_nav2_control.joint_state_mapping import (
    JOINT_NAMES,
    extract_joint_values,
)


def build_joint_state(motor_states, stamp):
    position, velocity, effort = extract_joint_values(motor_states)
    message = JointState()
    message.header.stamp = stamp
    message.name = list(JOINT_NAMES)
    message.position = position
    message.velocity = velocity
    message.effort = effort
    return message


class LowStateJointStateBridge(Node):
    def __init__(self):
        super().__init__('go2_lowstate_joint_state_bridge')
        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('joint_states_topic', '/joint_states')

        lowstate_topic = self.get_parameter(
            'lowstate_topic'
        ).get_parameter_value().string_value
        joint_states_topic = self.get_parameter(
            'joint_states_topic'
        ).get_parameter_value().string_value

        lowstate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            joint_states_topic,
            10,
        )
        self._lowstate_subscription = self.create_subscription(
            LowState,
            lowstate_topic,
            self._lowstate_callback,
            lowstate_qos,
        )
        self.get_logger().info(
            f'Publishing live Go2 joints: {lowstate_topic} -> '
            f'{joint_states_topic}'
        )

    def _lowstate_callback(self, message):
        try:
            joint_state = build_joint_state(
                message.motor_state,
                self.get_clock().now().to_msg(),
            )
        except ValueError as error:
            self.get_logger().warning(
                str(error),
                throttle_duration_sec=5.0,
            )
            return

        self._joint_state_publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = LowStateJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
