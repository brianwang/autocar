"""Autocar state machine — manages system modes."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from enum import Enum


class SystemState(Enum):
    BOOTUP = "BOOTUP"
    IDLE = "IDLE"
    AUTONOMY = "AUTONOMY"
    PAUSED = "PAUSED"
    ESTOP = "ESTOP"


class StateMachine(Node):
    VALID_TRANSITIONS = {
        SystemState.BOOTUP: [SystemState.IDLE],
        SystemState.IDLE: [SystemState.AUTONOMY],
        SystemState.AUTONOMY: [SystemState.PAUSED, SystemState.ESTOP],
        SystemState.PAUSED: [SystemState.AUTONOMY, SystemState.ESTOP, SystemState.IDLE],
        SystemState.ESTOP: [SystemState.IDLE],
    }

    def __init__(self):
        super().__init__('state_machine')
        self.state = SystemState.BOOTUP

        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.on_voice_command, 10)
        self.safety_sub = self.create_subscription(
            String, '/safety_state', self.on_safety_state, 10)
        self.task_pub = self.create_publisher(
            PoseStamped, '/task_goal', 10)
        self.state_pub = self.create_publisher(
            String, '/system_state', 10)

        self.create_timer(1.0, self.publish_state)
        self._switch_to(SystemState.IDLE)
        self.get_logger().info('state_machine initialized in IDLE')

    def _switch_to(self, target: SystemState) -> bool:
        if target in self.VALID_TRANSITIONS.get(self.state, []):
            old = self.state
            self.state = target
            self.get_logger().info(f'state transition: {old.value} -> {target.value}')
            return True
        self.get_logger().warn(f'invalid transition: {self.state.value} -> {target.value}')
        return False

    def on_voice_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'voice command in state {self.state.value}: {cmd}')

        if cmd == 'NAVIGATE' and self.state == SystemState.IDLE:
            self._switch_to(SystemState.AUTONOMY)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            self.task_pub.publish(goal)
        elif cmd == 'PAUSE' and self.state == SystemState.AUTONOMY:
            self._switch_to(SystemState.PAUSED)
        elif cmd == 'RESUME' and self.state == SystemState.PAUSED:
            self._switch_to(SystemState.AUTONOMY)
        elif cmd == 'STOP' and self.state in (SystemState.AUTONOMY, SystemState.PAUSED):
            self._switch_to(SystemState.IDLE)

    def on_safety_state(self, msg: String):
        if msg.data == 'estop_active' and self.state != SystemState.ESTOP:
            self._switch_to(SystemState.ESTOP)
            self.get_logger().error('ESTOP triggered!')

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
