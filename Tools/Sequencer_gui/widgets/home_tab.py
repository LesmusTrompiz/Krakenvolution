########################
# -- PESTAÑA INICIO -- #
########################
from PyQt5.QtWidgets import QWidget, QVBoxLayout


class HomeTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		self.layout = QVBoxLayout()

		# END: Set layout
		self.setLayout(self.layout)

