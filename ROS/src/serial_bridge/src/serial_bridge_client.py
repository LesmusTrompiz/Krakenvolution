import rclpy
from rclpy.action import ActionClient
from rclpy.node   import Node

from serial_bridge.action import Order


class OrderClient(Node):

    def __init__(self):
        super().__init__('order_action_client')
        self._action_client = ActionClient(self, Order, 'serial_bridge_server')

    def send_goal(self, order):
        self.get_logger().info('Sending Goal')
        goal_msg = order
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = OrderClient()
    future = action_client.send_goal(Order.Goal())
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()