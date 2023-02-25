from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLCDNumber


############################
# -- PESTAÑA PUNTUACIÓN -- #
############################

class PointsTab(QWidget):
	def __init__(self):
		super().__init__()


		# START: Define layout
		self.layout = QVBoxLayout()

		# Etiqueta wip
		self.lcdPoints = QLCDNumber()
		self.layout.addWidget(self.lcdPoints)

		# END: Set layout
		self.setLayout(self.layout)

	# ROS Callbacks
	def updatePoints(self, msg):
		self.lcdPoints.display(msg.data)