import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from serial_bridge_actions.action import Order
from yaml import load, SafeLoader
from std_srvs.srv import Empty

def load_yaml_routine(file : str) -> list:
    yaml_file = open(file, 'r')
    yaml_content = load(yaml_file, SafeLoader)
    routine = []
    start = 0
    while start < len(yaml_content):
        routine += [(yaml_content[start], yaml_content[start+1], yaml_content[start+2])]
        start += 3
    yaml_file.close()
    return routine

class SequencerNode(Node):
    def __init__(self):
        super().__init__('order_action_client')
        self.orders = []
        self.request_flag = False
        self.action_in_progress = False
        timer_period = 0.5  # seconds

        self.srv = self.create_service(Empty, 'load_sequence', self.update_request)
        self._action_client = ActionClient(self, Order, 'serial_bridge_server')
        self.timer = self.create_timer(timer_period, self.read_and_execute)
        
    def update_request(self,request, response):
        self.request_flag = True
        return response

    def send_goal(self, order):
        self.get_logger().info('Waiting Server')
        self.action_in_progress = True
        self._action_client.wait_for_server()
        self.get_logger().info('Sending Goal')
        self._send_goal_future = self._action_client.send_goal_async(order)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def read_and_execute(self):
        if not self.request_flag : return    
        if not self.orders:
            try:
                self.orders = load_yaml_routine("/media/pendrive/sequence_generated.yaml")
            except:
                self.get_logger().info('Not sequence found')
        else:
            self.get_logger().info('Yalm loaded')
            if self.action_in_progress: return
            order = self.orders.pop(0)
            device,id,arg = order
            ros_order = Order.Goal()
            ros_order.device = device
            ros_order.id = id
            ros_order.id = id
            ros_order.arg = arg
            self.send_goal(ros_order)
            if not self.orders: self.request_flag = False
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.ret))
        self.action_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    sequencer_node = SequencerNode()
    rclpy.spin(sequencer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sequencer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




