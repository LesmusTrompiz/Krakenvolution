from serial    import Serial
from threading import Thread
from time      import sleep


def configure_port(port_name : str) -> Serial:
    '''
        configure_port :: str -> Serial | SerialException
    '''
    BD = 115200
    port = Serial(port_name,BD,timeout=0.075)
    return port

def write(port : Serial ,msg : str):
    '''
        write_port :: SerialPort -> IO()
    '''
    msg += "\r"
    port.write(msg.encode()) 

def read(port : Serial) -> str:
    '''
        write_port :: SerialPort() -> String
    '''
    msg = port.readline()
    msg = msg.decode(encoding = 'utf-8')
    response = ""
    for letter in msg:
        if letter != "\r" and letter != "\n":
            response += letter
    return response


class serial_manager(Thread):
    # overriding constructor
    def __init__(self, dev_name : str):
        self.exit         = False
        self.port         = configure_port(dev_name)
        self.sent_msg     = None
        self.listen_msgs  = []
        # Calling parent class constructor
        Thread.__init__(self)
        
    # define your own run method
    def run(self):
        while not self.exit:
            if self.sent_msg:
                write(self.port,self.sent_msg)
                self.sent_msg = None
            else:
                response = read(self.port)
                if response:
                    self.listen_msgs += [response]
                sleep(0.05)

def write_to_port(port_mager : serial_manager ,msg : str):
    if port_mager.sent_msg == None:
        port_mager.sent_msg = msg
    else:
        raise ValueError("NOT ACOMPLISHED SENT")