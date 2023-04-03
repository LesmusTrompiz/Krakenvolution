from PyQt5.QtWidgets   import *
from .SimModesLayout   import SimModesLayout
from .ParametersWidget import ParametersLayout


############################
# -- PESTAÑA PARÁMETROS -- #
############################
		
class SimuladorTab(QWidget):
    def __init__(self, modes):
        super().__init__()
        clear_pb = QPushButton("Clear Obstacles")
        clear_pb.setStyleSheet("background-color: #3f3f3f")
        clear_pb.clicked.connect(self.clear_cb)
        self.clear_state = True

        # Set parameters
        set_parameter_pb = QPushButton("Set parameters")
        set_parameter_pb.setStyleSheet("background-color: #3f3f3f")
        set_parameter_pb.clicked.connect(self.set_parameter_cb)
        self.set_parameter_flag = False
        self.ModeWidget = SimModesLayout("Modos", modes)

		# START: Define layout
        self.SimLayout = QVBoxLayout()
        params = [("play_side",     ["default","blue", "green"]), 
                  ("spawn",         ["default", "1", "2", "3", "4", "5"]),
                  ("tree",          ["default", "hello_tree"]),
                  ("ally_tree",     ["default", "1", "2", "3", "4", "5"]),
                  ("start",         ["default", "true"])                  ]
        
        # Modes layout
        self.ParameterWidget = ParametersLayout("Parámetros ROS (Cliente del decision main)", params)
        #self.StartCB = QCheckBox("Start Signal")
        self.SimLayout.addWidget(self.ModeWidget)
        self.SimLayout.addWidget(clear_pb)
        self.SimLayout.addWidget(self.ParameterWidget)
        self.SimLayout.addWidget(set_parameter_pb)
        #self.SimLayout.addWidget(self.StartCB)

        self.setLayout(self.SimLayout)

    def get_mode(self):
        return self.ModeWidget.mode
    
    def clear_cb(self):
        self.clear_state = True

    def get_clear_state(self):
        flag = self.clear_state
        self.clear_state = False
        return flag

    def set_parameter_cb(self):
        self.set_parameter_flag = True

    def get_set_parameter_state(self):
        flag = self.set_parameter_flag
        self.set_parameter_flag = False
        return flag

    def get_paremeters(self):
        return self.ParameterWidget.params
    
    def reset_params(self):
        self.ParameterWidget.reset_params()
        #self.StartCB.setChecked(False)
        return 
    