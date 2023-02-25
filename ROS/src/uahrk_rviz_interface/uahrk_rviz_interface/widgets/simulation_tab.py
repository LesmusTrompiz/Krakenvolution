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
		# START: Define layout
        self.SimLayout   	     = QVBoxLayout()
        pbs = [(f"pb{n}", f"pb{n}") for n in range(3)]
        self.ModeWidget = SimModesLayout("Modos", modes)
        self.SimLayout.addWidget(self.ModeWidget)
        self.SimLayout.addWidget(clear_pb)
        self.SimLayout.addWidget(self.ModeWidget)
        self.SimLayout.addWidget(ParametersLayout("Texto", pbs))
        self.setLayout(self.SimLayout)

    def get_mode(self):
        return self.ModeWidget.mode
    def clear_cb(self):
        self.clear_state = True

    def get_clear_state(self):
        flag = self.clear_state
        self.clear_state = False
        return flag
    