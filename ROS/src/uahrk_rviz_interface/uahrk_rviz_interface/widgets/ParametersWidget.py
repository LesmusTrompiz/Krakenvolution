from PyQt5.QtCore	 	  import Qt
from PyQt5.QtGui	 	  import QIntValidator
from PyQt5.QtWidgets 	  import *
from typing               import List,Tuple
from functools            import partial

class ParametersLayout(QWidget):
    def __init__(self, title : str, params : List[Tuple[str,str]]):
        super().__init__()
        self.layout         = QVBoxLayout()
        self.ParamsHeader   = QHBoxLayout()
        self.ParamsCB   	= QHBoxLayout()
        self.params         = {}

		# Titulo
        self.exLabel = QLabel(title)
        self.exLabel.setAlignment(Qt.AlignCenter)
        self.exLabel.setStyleSheet("background-color: #00007f; font-weight: bold; color: white; border-radius: 10px;")
        self.exLabel.setMaximumHeight(30)
        self.ParamsHeader.addWidget(self.exLabel)
        


        # PushButtons
        self.buttons = []
        for param in params:
            block_title_box = QVBoxLayout()
            id, posibble_values = param
            title = QLabel(id)
            button = QComboBox()
            button.addItems(posibble_values)
            button.setStyleSheet("background-color: #3f3f3f")
            button.currentTextChanged.connect(partial(self.cb_pb,id))
            self.buttons += [button]
            block_title_box.addWidget(title)
            block_title_box.addWidget(button)
            self.ParamsCB.addLayout(block_title_box)

            self.params[id] = posibble_values[0]

        
        # Header
        self.layout.addLayout(self.ParamsHeader)
        self.layout.addLayout(self.ParamsCB)
        self.setLayout(self.layout)

    def cb_pb(self,service_name , value):
        self.params[service_name] = value

    def get_parameters(self):
        return self.params
    
    def reset_params(self):
        self.params = {}
        for button in self.buttons:
            button.setCurrentIndex(0)
        return 