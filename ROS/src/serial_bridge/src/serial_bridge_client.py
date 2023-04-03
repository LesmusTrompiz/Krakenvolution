'''
    This node acts as a simple client of the Serial 
    Bridge Node. This node creates an Action client
    to the serial_bridge server and ask to the user
    some commands to send.
''' 

import rclpy
from rclpy.action import ActionClient
from rclpy.node   import Node
from serial_bridge_actions.action import Order



class OrderClient(Node):
    '''
        This node acts as a simple client of the Serial 
        Bridge Node. This node creates an Action client
        to the serial_bridge server and ask to the user
        some commands to send.
    '''
    def __init__(self):
        '''
            Initializes the node and the Action
            Servers.
        '''
        super().__init__('order_action_client')
        self._action_client = ActionClient(self, Order, 'serial_bridge_server')
        if not self._action_client.wait_for_server(2):
            self.get_logger().error("No se ha podido conectar con el servidor")
            raise()
    def send_goal(self, order):
        '''
            Sends the order to the action server
        '''
        self.get_logger().info('Sending Goal')        
        goal_msg = order
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    

    def user_loop(self):
        '''
            Is the interface with the user, ask for 
            orders until the user is bored then it
            returns.
        '''
        exit_ = int(input("Exit? - 0: To exit - Any other number : Keep sending orders "))
        while exit_ != 0:
            g     = Order.Goal()
            g.id  = input("Order ID? ")
            g.arg = int(input("Order ARG? "))
            self.send_goal(g)
            exit_ = int(input("Exit? - 0: To exit - Any other number : Keep sending orders "))
        return


def main(args=None):
    rclpy.init(args=args)
    action_client = OrderClient()
    action_client.user_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()