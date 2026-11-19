import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from truckbot_msgs.msg import Heartbeat, SystemHealth

from dashboard.dashboard import Dashboard

STATE_NAMES = {
    Heartbeat.INIT: 'INIT',
    Heartbeat.GOOD: 'GOOD',
    Heartbeat.DEGRADED: 'DEGRADED',
    Heartbeat.FAILED: 'FAILED',
}


SERVICE_TIMEOUT_S = 2.0


class DashboardMiddleware(Node):

    def __init__(self, dashboard):
        super().__init__('dashboard')
        self.dashboard = dashboard
        self.port = self.declare_parameter('port', 8080).value
        self.create_subscription(SystemHealth, '/health/system', self.system_health_callback, 10)
        self.start_client = self.create_client(Trigger, '/mission/start')

    def deliver(self):
        # Runs on a Flask thread; the executor thread completes the future
        if not self.start_client.service_is_ready():
            return False, '/mission/start not available'
        done = threading.Event()
        future = self.start_client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(SERVICE_TIMEOUT_S):
            future.cancel()
            return False, '/mission/start timed out'
        result = future.result()
        if result is None:
            return False, '/mission/start call failed'
        self.get_logger().info(f'Deliver: {result.message}')
        return result.success, result.message

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
    dashboard.deliver = node.deliver
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
