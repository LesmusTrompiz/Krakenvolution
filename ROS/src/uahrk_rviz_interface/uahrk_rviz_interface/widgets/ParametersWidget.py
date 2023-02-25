from PyQt5.QtCore	 	  import Qt
from PyQt5.QtGui	 	  import QIntValidator
from PyQt5.QtWidgets 	  import *
from copy import deepcopy
from typing import List,Tuple
from functools import partial
class ParametersLayout(QWidget):
    def __init__(self, title : str, modes : List[Tuple[str,str]]):
        super().__init__()
        self.layout      = QVBoxLayout()
        self.ParamsHeader   	 = QHBoxLayout()
        self.ParamsCB   	     = QHBoxLayout()
        self.text = []

		# Titulo
        self.exLabel = QLabel(title)
        self.exLabel.setAlignment(Qt.AlignCenter)
        self.exLabel.setStyleSheet("background-color: #00007f; font-weight: bold; color: white; border-radius: 10px;")
        self.exLabel.setMaximumHeight(30)
        self.ParamsHeader.addWidget(self.exLabel)
        self.buttons = []
        # PushButtons
        
        for mode in modes:
            pb_text, serv_call = mode
            button = QCheckBox(pb_text)
            button.setStyleSheet("background-color: #3f3f3f")
            button.clicked.connect(partial(self.cb_pb,serv_call))
            self.ParamsCB.addWidget(button)
        
        # Header
        self.layout.addLayout(self.ParamsHeader)
        self.layout.addLayout(self.ParamsCB)
        self.setLayout(self.layout)

    def cb_pb(self,n : str):
        print(f"Llamand al servicio {n}")

