#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets        	import *
from widgets.simulation_tab    import SimuladorTab
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QTimer
from rclpy.action import ActionClient
from uahrk_navigation_msgs.action import GoToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from uahrk_navigation_msgs.srv import SetPose2d
from tf_transformations import euler_from_quaternion
from math import pi


WIDTH  = 1024
HEIGHT = 600

##########################
# -- CLASE APLICACIÓN -- #
##########################

# Creating the main window 
class Application(QMainWindow): 
	def __init__(self): 
		print("a")
		# QT things
		QMainWindow.__init__(self)
		self.setWindowTitle('UAHR Krakens Team - Eurobot 2023') 
		self.setGeometry(0, 0, WIDTH, HEIGHT) 

		self.tab_manager = TabManager(self) 
		self.setCentralWidget(self.tab_manager) 

		return

############################
# -- GESTOR DE PESTAÑAS -- #
############################

# Creating tab widget
class TabManager(QWidget): 
	def __init__(self, parent): 
		super(QWidget, self).__init__(parent) 
		self.layout = QVBoxLayout(self) 

		# Initialize tab screen 
		self.tabM = QTabWidget()
		self.tabM.resize(WIDTH, HEIGHT) 

		# Add tabs, choose icon and name
		#self.tabM.addTab(HomeTab(), QIcon("img/boat.png"),("Inicio")) 
		#self.tabM.addTab(ParamTab(), QIcon("img/compass.png"), ("Parámetros"))
		#self.tabM.addTab(PointsTab(), QIcon("img/crab.png"), ("Puntuación")) 
		self.tabM.addTab(SimuladorTab(), "Simulador") 

		# Add tabs to widget 
		self.layout.addWidget(self.tabM) 
		self.setLayout(self.layout)


#######################
# -- MAIN FUNCTION -- #
#######################

if __name__ == '__main__': 
	app  = QApplication(sys.argv) 
	print("hola")


	root = Application() 
	root.show()
	#rclpy.spin(app)
	print("hola")
	sys.exit(app.exec_()) 
