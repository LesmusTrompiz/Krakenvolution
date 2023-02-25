from PyQt5.QtCore	 	  import Qt
from PyQt5.QtGui	 	  import QIntValidator
from PyQt5.QtWidgets 	  import *
from copy import deepcopy
from typing import List,Tuple
from functools import partial
class SimModesLayout(QWidget):
    def __init__(self, title : str,  modes : List[Tuple[str,str]]):
        super().__init__()
        self.layout      = QVBoxLayout()
        self.SimModesHeader   	 = QHBoxLayout()
        self.SimModesPbs   	     = QHBoxLayout()
        self.text = []

        self.mode = "undefined"
		# Titulo
        self.exLabel = QLabel(title)
        self.exLabel.setAlignment(Qt.AlignCenter)
        self.exLabel.setStyleSheet("background-color: #00007f; font-weight: bold; color: white; border-radius: 10px;")
        self.exLabel.setMaximumHeight(30)
        self.SimModesHeader.addWidget(self.exLabel)
        self.buttons = []
        # PushButtons
        for mode in modes:
            pb_text, serv_call = mode
            button = QPushButton(pb_text)
            button.setStyleSheet("background-color: #3f3f3f")
            button.clicked.connect(partial(self.cb_pb,serv_call))
            self.SimModesPbs.addWidget(button)
        
        # Header
        self.layout.addLayout(self.SimModesHeader)
        self.layout.addLayout(self.SimModesPbs)
        self.setLayout(self.layout)

    def cb_pb(self,n):
        print(f"Modo {n}")
        self.mode = n

