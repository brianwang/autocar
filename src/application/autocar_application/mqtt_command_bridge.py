"""MQTT command bridge — subscribe to remote MQTT, publish ROS2 commands."""

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import paho.mqtt.client as mqtt
except ImportError:
    raise ImportError("paho-mqtt not installed. Run: pip install paho-mqtt")


class MqttCommandBridge(Node):
    """Bridge MQTT → ROS2 for remote car control."""

    DEFAULT_SPEED = 0.3       # m/s
    DEFAULT_TURN = 0.5        # rad/s

    def __init__(self):
        super().__init__('mqtt_command_bridge')

        # === params ===
        self.declare_parameter('mqtt_host', '124.221.233.228')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_user', 'wyz')
        self.declare_parameter('mqtt_pass', '123456')
        self.declare_parameter('mqtt_topic', 'test/topic')
        self.declare_parameter('mqtt_client_id', 'autocar_bridge')

        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        user = self.get_parameter('mqtt_user').value
        pwd  = self.get_parameter('mqtt_pass').value
        topic = self.get_parameter('mqtt_topic').value
        cid  = self.get_parameter('mqtt_client_id').value

        # === publishers ===
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._voice_pub = self.create_publisher(String, '/voice_command', 10)

        # === MQTT setup ===
        self._client = mqtt.Client(client_id=cid)
        self._client.username_pw_set(user, pwd)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        self._client.connect_async(host, port, keepalive=60)
        self._client.loop_start()
        self._topic = topic

        self.get_logger().info(
            f'mqtt bridge starting — {host}:{port} topic={topic}'
        )

    # ── MQTT callbacks ──────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self._topic, qos=1)
            self.get_logger().info(f'mqtt connected, subscribed {self._topic}')
        else:
            self.get_logger().error(f'mqtt connect failed, rc={rc}')

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f'mqtt rx: {payload}')
        except Exception as e:
            self.get_logger().warn(f'decode error: {e}')
            return

        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = {'cmd': payload.strip().lower()}

        self._handle_command(data)

    # ── command dispatch ────────────────────────────────────────

    def _handle_command(self, data: dict):
        cmd = data.get('cmd', '').strip().lower()
        lx  = data.get('linear_x')
        az  = data.get('angular_z')

        # ── named commands ──
        if cmd in ('forward', 'f', '前进'):
            self._publish_twist(lx or self.DEFAULT_SPEED, 0.0)
        elif cmd in ('backward', 'b', '后退'):
            self._publish_twist(-(lx or self.DEFAULT_SPEED), 0.0)
        elif cmd in ('left', 'l', '左转'):
            self._publish_twist(0.0, az or self.DEFAULT_TURN)
        elif cmd in ('right', 'r', '右转'):
            self._publish_twist(0.0, -(az or self.DEFAULT_TURN))
        elif cmd in ('stop', 's', '停止'):
            self._publish_twist(0.0, 0.0)
        elif cmd in ('voice', 'v', '语音'):
            text = data.get('text', '')
            if text:
                msg = String()
                msg.data = text
                self._voice_pub.publish(msg)
                self.get_logger().info(f'voice cmd: {text}')

        # ── raw velocity ──
        elif lx is not None or az is not None:
            self._publish_twist(lx or 0.0, az or 0.0)

        else:
            self.get_logger().warn(f'unknown command: {data}')

    def _publish_twist(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)
        self.get_logger().info(
            f'cmd_vel: linear={linear_x:.2f} angular={angular_z:.2f}'
        )

    # ── lifecycle ───────────────────────────────────────────────

    def destroy_node(self):
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
