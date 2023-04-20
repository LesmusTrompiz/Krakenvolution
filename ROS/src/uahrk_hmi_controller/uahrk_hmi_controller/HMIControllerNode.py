import rclpy
from rclpy.client import SrvTypeRequest
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,  ParameterType, ParameterValue
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import UInt16 
import serial
import json
from typing import Dict, List
from os import system, scandir 
from os.path import isfile
from sys import argv, stdout
import threading
import string

def reboot():
    system('reboot')

def halt():
    system('halt')

TREES_FOLDER = '/ros_ws/ROS/src/uahrk_decision_making/behavior_trees/'

class HMIControllerNode(Node):
    def __init__(self, device_port : str, trees: List[str]):
        super().__init__('hmi_controller')
        # Internal logic
        self.msg_queue = []
        self.pending_ack = False
        
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
        self.serial_device       = serial.Serial(device_port, 115200) 

        print('Waiting for serial', end='')
        stdout.flush()
        
        port_open = False
        while not port_open:
            msg = self.serial_device.read_until(b'\n').decode('ascii')
            print(f'Read: {msg}')
            if 'ready' in msg:
                print('HMI ready!')
                self.serial_device.write(b'ok\n')
                port_open = True
            stdout.flush()


        # Send trees to device
        for tree in trees:
            self.send_msg('plan', tree)

    
        # ROS Interface:
        self.set_params_client   = self.create_client(SetParameters, "/DecisionTree/set_parameters")
        self.srv_reset_params    = self.create_service(Empty, '/reset_params', self.cb_reset_params)
        self.srv_pendrive_state  = self.create_service(SetBool, 'pendrive_status', self.cb_pendrive_state)
        self.points_subscriber   = self.create_subscription(UInt16, '/points', self.cb_points, 10)

        # ROS Timer to execute the tick function:
        timer_period = 0.1
        self.timer   = self.create_timer(timer_period, self.tick_node)

        # Launch reader thread
        self.read_thread = threading.Thread(target=self.read_port)
        self.read_thread.daemon = True
        self.read_thread.start()


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

    def cb_pendrive_state(self,request : SetBool.Request, response : SetBool.Response):
        '''
            This function will be called when a request 
            is received throught '/pendrive_status' service.

            This function may assing the request value
            to the pendrive state attribute.
        '''

        self.send_msg('pendrive', str(request.data).lower())
        self.get_logger().info(f'pendrive: {str(request.data).lower()}')
        stdout.flush()
        self.pendrive_pluged = request.data
        response.success = True
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
        while True:
            msg = self.serial_device.read_until(b'\n').decode('utf8').replace('\n', '').replace('\r', '')
            # Check if it's ACK or NACK
            if self.pending_ack:
                if 'ok' == msg:
                    if len(self.msg_queue) != 0:
                        # Pop from the front of the message queue
                        next_msg = self.msg_queue[0]
                        self.msg_queue = self.msg_queue[1:]
                        self.serial_device.write(next_msg)
                        print(f'Sending message: {next_msg}')
                    else:
                        self.pending_ack = False
                    continue
                elif msg == 'nok':
                    print('Last message was invalid!')
                    continue

            print(f'msg: {msg}')
            stdout.flush()
            try:
                decoded = json.loads(msg)
                print(f'Decoded msg: {decoded}')
                self.send_data(decoded)
                # Get actions
                parameters, actions = self.analize_msg(decoded)
            except Exception as e:
                print(f"Failed to decode message: {msg}")
                print(e)
            finally:
                msg = ''
            stdout.flush()
        return
    
    def send_msg(self, command : str, arg : str):
        msg = f'{command}:{arg}\n'.encode('ascii')
        if not self.pending_ack:
            self.pending_ack = True
            self.serial_device.write(msg)
            print(f'Sending message directly: {msg}')
        else:
            self.msg_queue.append(msg)
            print(f'Adding {msg} to queue')

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
    def send_data(self, data):
        tree_param =            Parameter(name='tree', value=ParameterValue(type=4, string_value=data['tree']))
        ally_tree_param =       Parameter(name='ally_tree', value=ParameterValue(type=4, string_value=''))
        spawn_param =           Parameter(name='spawn', value=ParameterValue(type=4, string_value=str(data['spawn'])))
        play_side_param =       Parameter(name='play_side', value=ParameterValue(type=4, string_value=data['play_side']))
        parking_side_param =    Parameter(name='parking_site', value=ParameterValue(type=4, string_value=str(data['parking'])))
        
        params = SetParameters.Request(parameters=[
            tree_param,
            ally_tree_param,
            spawn_param,
            play_side_param,
            parking_side_param
        ])
        self.set_params_client.call(params)

def main(args=None):
    rclpy.init(args=args)
    # Check if port was passed by args
    if len(argv) < 2:
        print('Serial port was not specified via args')
        exit(1)

    # Open folder and get trees
    trees = []
    with scandir(TREES_FOLDER) as files:
        for file in files:
            if isfile(file):
                if file.path.endswith('.tree.xml'):
                    trees.append(file.name[:-len('.tree.xml')])

    for tree in trees:
        print(f'Found tree: {tree}')

    device_port = argv[1]
    print(f'Opening serial port {device_port}...')

    hmi_controller_node = HMIControllerNode(device_port, trees)
    rclpy.spin(hmi_controller_node)

    # Destroy the node explicitly
    hmi_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





