#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtGui	        	import QIcon
from PyQt5.QtWidgets        	import *
from widgets.secuenciador_tab   import SecuenciadorTab




WIDTH  = 1024
HEIGHT = 600

##########################
# -- CLASE APLICACIÓN -- #
##########################

# Creating the main window 
class Application(QMainWindow): 
	def __init__(self): 
		super().__init__()
		self.setWindowTitle('UAHR Krakens Team - Eurobot 2023') 
		self.setGeometry(0, 0, WIDTH, HEIGHT) 

		self.tab_manager = TabManager(self) 
		self.setCentralWidget(self.tab_manager) 

		self.show() 

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
		self.tabM.addTab(SecuenciadorTab(), "Secuenciador") 


		# Add tabs to widget 
		self.layout.addWidget(self.tabM) 
		self.setLayout(self.layout)

#######################
# -- MAIN FUNCTION -- #
#######################

if __name__ == '__main__': 
	app  = QApplication(sys.argv) 
	root = Application() 
	sys.exit(app.exec_()) 
