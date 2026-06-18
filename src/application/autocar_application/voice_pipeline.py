"""Voice pipeline — VAD → Wake Word → ASR(cloud) → LLM(cloud) → Intent → TTS(cloud)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


INTENT_RULES = [
    ("去", "NAVIGATE"),
    ("到", "NAVIGATE"),
    ("停", "PAUSE"),
    ("走", "OVERRIDE_PLAN"),
    ("绕", "REPLAN"),
    ("多远", "QUERY"),
    ("继续", "RESUME"),
    ("回", "NAVIGATE"),
]


class VoicePipeline(Node):
    def __init__(self):
        super().__init__('voice_pipeline')
        self.cmd_pub = self.create_publisher(String, '/voice_command', 10)
        self.tts_text_pub = self.create_publisher(String, '/tts_text', 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('voice_pipeline initialized')

    def parse_intent(self, text: str) -> str:
        for keyword, intent in INTENT_RULES:
            if keyword in text:
                self.get_logger().info(f'intent matched: {intent} via keyword "{keyword}"')
                return intent
        return "UNKNOWN"

    async def process_utterance(self, text: str):
        """Process natural language utterance through the pipeline."""
        intent = self.parse_intent(text)

        if intent == "UNKNOWN":
            self.get_logger().warn(f'unrecognized utterance: {text}')
            return

        cmd = String()
        cmd.data = intent
        self.cmd_pub.publish(cmd)

        if intent == "NAVIGATE":
            response = "好的，开始导航"
        elif intent == "PAUSE":
            response = "已暂停"
        elif intent == "RESUME":
            response = "继续行驶"
        elif intent == "REPLAN":
            response = "重新规划路径"
        else:
            response = "收到"

        tts = String()
        tts.data = response
        self.tts_text_pub.publish(tts)

    def _heartbeat(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VoicePipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
