import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,  ParameterType
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import UInt16
import serial
import json
from typing import Dict
from os import system
from sys import argv
import threading

def reboot():
    system('reboot')

def halt():
    system('halt')



class HMIControllerNode(Node):
    def __init__(self, device_port : str):
        super().__init__('hmi_controller')
        
        # Own Attributes:
        self.params              = {}
        self.pendrive_pluged     = False
        self.points              = 0
        self.points_update       = False

        self.ros_parameters      = ['play_side',
                                    'spawn',
                                    'tree',
                                    'ally_tree']

        self.possible_actions    = {'reboot' : reboot,
                                    'halt'   : halt}

        # Initialize the serial protocol:
        self.serial_device       = serial.Serial(device_port, 112500) 

    
        # ROS Interface:
        self.set_params_client   = self.create_client(SetParameters, "/DecisionTree/set_parameters")
        self.srv_reset_params    = self.create_service(Empty, '/reset_params', self.cb_reset_params)
        self.srv_pendrive_state  = self.create_service(SetBool, '/pendrive_status', self.cb_pendrive_state)
        self.points_subscriber   = self.create_subscription(UInt16, '/points', self.cb_points, 10)

        # ROS Timer to execute the tick function:
        timer_period = 0.1
        self.timer   = self.create_timer(timer_period, self.tick_node)

        # Launch reader thread
        read_thread = threading.Thread(target=self.read_port)



    def cb_reset_params(self, request : Empty.Request, response : Empty.Response):
        '''
            This function will be called when a request 
            is received throught '/reset_params' service.

            This function may reset all the stored 
            parameters.
        '''
        # To restore the parameters just assing an empty
        # dictionary
        self.params = {}
        return response

    def cb_pendrive_state(self,request : SetBool.Request, response):
        '''
            This function will be called when a request 
            is received throught '/pendrive_status' service.

            This function may assing the request value
            to the pendrive state attribute.
        '''

        self.send_msg('pendrive', str(request.data).lower())
        self.pendrive_pluged = request.data
        return response

    def cb_points(self, points : UInt16):
        '''
        
        
        '''
        self.points = points.data
        self.points_update = True
        self.send_msg('score', str(self.points))
        

    def update_params(self, params : Dict[str, str]):
        '''
        
        
        '''
        # Create the Request msg that will be set 
        # all the parameters of the other node
        parameters_msg = SetParameters.Request()

        # For every parameter create a paramerter_msg
        # that will store the parameter value as a 
        # string
        for key in params.keys():
            param_msg = Parameter()
            param_msg.name = key
            param_msg.value.string_value = params[key]
            param_msg.value.type = ParameterType.PARAMETER_STRING
            parameters_msg.parameters.append(param_msg)
        
        # Send the request msg
        self.set_params_client.call_async(parameters_msg)

    def read_port(self):
        self.serial_device.timeout = None
        read = ''
        while not self.get_node().is_shutdown():
            msg = self.serial_device.read().decode('ascii')
            print(f'msg: {msg}')
            if msg == '\n' or msg == '\0':
                try:
                    decoded = json.loads(read)
                    print(f'Decoded msg: {decoded}')
                    # Get actions
                    parameters, actions = self.analize_msg(decoded)
                except Exception as e:
                    print(f"Failed to decode message: {read}")
                    print(e)
                finally:
                    msg = ''
            else:
                read += msg
        return
    
    def send_msg(self, command : str, arg : str):
        self.serial_device.write(f'{command}:{arg}\n'.encode('ascii'))

    def analize_msg(self, msg : Dict[str, str]):
        
        parameters = {}
        actions    = []
        for id in msg.keys():
            
            # If the msg is one of the needed params
            # store the value in params
            if id in self.ros_parameters: 
                parameters[id] = msg[id]
            
            if id in self.possible_actions.keys():
                actions += [self.possible_actions[id]]

        return parameters, actions

    def tick_node(self):
        pass
        # msg = self.read_port()
        # if msg:
        #     decode_json     = json.load(msg)
        #     params, actions = self.analize_msg(decode_json)
        #     self.update_params(params)
        #     for action in actions:
        #         action()
        #
        # if self.points_update: # Creo que esto es lo que querías hacer
        #                         # Pero te lo pongo en el propio callback
        #     self.points_update = False
        #     point_msg = json.dumps({'points' : self.points})
        #     self.send_msg('points', str(self.points))

def main(args=None):
    rclpy.init(args=args)
    # Check if port was passed by args
    if len(argv) < 2:
        print('Serial port was not specified via args')
        exit(1)

    device_port = argv[1]

    hmi_controller_node = HMIControllerNode(device_port)
    rclpy.spin(hmi_controller_node)

    # Destroy the node explicitly
    hmi_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





