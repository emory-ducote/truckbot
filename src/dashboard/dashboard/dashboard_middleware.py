import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from truckbot_msgs.msg import Heartbeat, SystemHealth

from dashboard.dashboard import Dashboard

STATE_NAMES = {
    Heartbeat.INIT: 'INIT',
    Heartbeat.GOOD: 'GOOD',
    Heartbeat.DEGRADED: 'DEGRADED',
    Heartbeat.FAILED: 'FAILED',
}


class DashboardMiddleware(Node):

    def __init__(self, dashboard):
        super().__init__('dashboard')
        self.dashboard = dashboard
        self.port = self.declare_parameter('port', 8080).value
        self.create_subscription(SystemHealth, '/health/system', self.system_health_callback, 10)

    def system_health_callback(self, msg):
        modules = [
            {
                'name': m.node_name,
                'state': STATE_NAMES.get(m.state, 'FAILED'),
                'detail': m.detail,
                'stamp': m.stamp.sec + m.stamp.nanosec * 1e-9,
            }
            for m in msg.modules
        ]
        self.dashboard.updateStatus(STATE_NAMES.get(msg.state, 'FAILED'), modules)


def main():
    # Leave SIGINT to Python so Ctrl+C stops Flask in the main thread
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    dashboard = Dashboard()
    node = DashboardMiddleware(dashboard)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info(f'Dashboard serving on http://0.0.0.0:{node.port}')
    try:
        dashboard.createApp().run(host='0.0.0.0', port=node.port)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
