import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,  ParameterType
from std_srvs.srv import Empty

class hmi_controller_node(Node):
    def __init__(self):
        Node.__init__(self,'Rviz_interface')
        self.keys              = {}
        self.set_params_client = self.create_client(SetParameters, "/DecisionTree/set_parameters")
        self.srv_reset_params  = self.create_service(Empty, 'reset_params', self.cb_reset_params)
    
    def cb_reset_params(self, request, response : Empty.Response):
        self.keys = {}
        return response

    def update_params(self):
        

