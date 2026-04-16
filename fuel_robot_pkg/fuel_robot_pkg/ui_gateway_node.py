import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class UiGatewayNode(Node):
    def __init__(self):
        super().__init__('ui_gateway_node')

        self.declare_parameter('http_port', 6000)
        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value

        self.start_pub = self.create_publisher(Bool, '/fueling/start', 10)
        self.xyz_pub = self.create_publisher(
            Float64MultiArray, '/fueling/fuel_port_xyz', 10
        )

        self.status_sub = self.create_subscription(
            String, '/fueling/status', self.status_callback, 10
        )
        self.done_sub = self.create_subscription(
            Bool, '/fueling/done', self.done_callback, 10
        )

        self.latest_status = ''
        self.latest_done = None

        self._start_http_server()
        self.get_logger().info(
            f'UI gateway ready. HTTP listening on port {self.http_port}'
        )

    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length else b'{}'
                try:
                    data = json.loads(body)
                except json.JSONDecodeError:
                    data = {}

                if self.path == '/fueling/start':
                    self._handle_start(data)
                elif self.path == '/fueling/estop':
                    self._handle_estop(data)
                else:
                    self._respond(404, {'error': 'not found'})

            def do_GET(self):
                if self.path == '/fueling/status':
                    self._respond(200, {
                        'status': node.latest_status,
                        'done': node.latest_done,
                    })
                else:
                    self._respond(404, {'error': 'not found'})

            def _handle_start(self, data):
                x = data.get('robot_x')
                y = data.get('robot_y')
                z = data.get('robot_z')

                if x is None or y is None or z is None:
                    self._respond(400, {'error': 'robot_x, robot_y, robot_z required'})
                    return

                start_msg = Bool()
                start_msg.data = True
                node.start_pub.publish(start_msg)
                node.get_logger().info('published /fueling/start')

                xyz_msg = Float64MultiArray()
                xyz_msg.data = [float(x), float(y), float(z)]
                node.xyz_pub.publish(xyz_msg)
                node.get_logger().info(
                    f'published xyz: [{x:.1f}, {y:.1f}, {z:.1f}]'
                )

                node.latest_done = None
                self._respond(200, {'ok': True})

            def _handle_estop(self, data):
                node.get_logger().warn('E-STOP received from web')
                node.publish_status('estop_from_web')
                self._respond(200, {'ok': True})

            def _respond(self, code, obj):
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(obj).encode())

            def log_message(self, format, *args):
                node.get_logger().debug(f'HTTP: {format % args}')

        server = HTTPServer(('0.0.0.0', self.http_port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub = self.create_publisher(String, '/fueling/status', 10)
        self.status_pub.publish(msg)

    def status_callback(self, msg: String):
        self.latest_status = msg.data
        self.get_logger().info(f'[UI STATUS] {msg.data}')

    def done_callback(self, msg: Bool):
        self.latest_done = msg.data
        if msg.data:
            self.get_logger().info('[UI STATUS] Fueling task completed.')
        else:
            self.get_logger().warn('[UI STATUS] Fueling task failed.')


def main(args=None):
    rclpy.init(args=args)
    node = UiGatewayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
