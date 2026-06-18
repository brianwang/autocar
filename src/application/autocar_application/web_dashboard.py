"""Web dashboard — serve status/map via HTTP for browser monitoring."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import json
import http.server
import socketserver
import threading


class DashboardHandler(http.server.BaseHTTPRequestHandler):
    state = "IDLE"
    position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    battery = 12.0
    path_points = []

    def do_GET(self):
        if self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = {
                "state": self.state,
                "position": self.position,
                "battery": self.battery,
                "path_length": len(self.path_points),
            }
            self.wfile.write(json.dumps(data).encode())
        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = self._build_html()
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def _build_html(self) -> str:
        return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Autocar Dashboard</title>
<style>
  body { font-family: sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
  .card { background: #16213e; border-radius: 8px; padding: 16px; margin: 8px 0; }
  .state { font-size: 24px; font-weight: bold; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; }
  canvas { background: #0f3460; border-radius: 8px; width: 100%; height: 300px; }
  h1 { color: #e94560; }
</style>
</head>
<body>
<h1>🚗 Autocar Dashboard</h1>
<div class="grid">
  <div class="card"><div class="state" id="state">IDLE</div><div>系统状态</div></div>
  <div class="card"><span id="battery">12.0</span>V<div>电池电压</div></div>
  <div class="card">X: <span id="x">0.00</span> Y: <span id="y">0.00</span><div>位置</div></div>
</div>
<div class="card"><canvas id="map"></canvas></div>
<script>
async function update() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    document.getElementById('state').textContent = d.state;
    document.getElementById('battery').textContent = d.battery.toFixed(1);
    document.getElementById('x').textContent = d.position.x.toFixed(2);
    document.getElementById('y').textContent = d.position.y.toFixed(2);
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#0f3460';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#e94560';
    const cx = canvas.width / 2 + d.position.x * 4;
    const cy = canvas.height / 2 - d.position.y * 4;
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, Math.PI * 2);
    ctx.fill();
  } catch(e) {}
}
setInterval(update, 500);
</script>
</body></html>"""


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.state_sub = self.create_subscription(
            String, '/system_state', self.on_state, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_fused', self.on_odom, 10)

        self._start_server()
        self.get_logger().info('web_dashboard running on http://0.0.0.0:8080')

    def on_state(self, msg: String):
        DashboardHandler.state = msg.data

    def on_odom(self, msg: Odometry):
        DashboardHandler.position = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": 0.0,
        }

    def _start_server(self):
        def _run():
            with socketserver.TCPServer(("0.0.0.0", 8080), DashboardHandler) as httpd:
                httpd.serve_forever()
        t = threading.Thread(target=_run, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
