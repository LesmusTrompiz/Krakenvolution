import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from serial_bridge.action import Order
from serial_manager import serial_manager
class FibonacciActionServer(Node):

    def __init__(self, device : str):
        super().__init__('serial_action_server')
        self.port = serial_manager(device)
        self._action_server = ActionServer(
            self,
            Order,
            'serial_bridge_server',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Sendig msg {goal_handle.request.id}')
        feedback_msg      = Order.Feedback()
        feedback_msg.sent = True


        goal_handle.publish_feedback(feedback_msg)
        
        # Wait until msg response check
        
        time.sleep(1)

        goal_handle.succeed()

        result = Order.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
