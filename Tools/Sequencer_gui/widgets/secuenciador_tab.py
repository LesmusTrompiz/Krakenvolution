
from PyQt5.QtCore	 	  import Qt
from PyQt5.QtGui	 	  import QIntValidator
from PyQt5.QtWidgets 	  import *
from utils.yaml_interface import load_yaml_actions, dump_yaml_routine, load_yaml_routine


############################
# -- PESTAÑA PARÁMETROS -- #
############################
		
class SecuenciadorTab(QWidget):
	def __init__(self):
		super().__init__()

		# START: Define layout
		#self.hLayout  = QHBoxLayout()
		#self.lLayout  = QVBoxLayout()
		#self.lbLayout = QHBoxLayout()
		self.loadedActions       = {} 
		self.reversedDict        = {} 

		self.SecLayout   	     = QVBoxLayout()
		self.addRoutineLayout 	 = QHBoxLayout()
		self.removeRoutineLayout = QHBoxLayout()
		self.outpoutLayout 		 = QHBoxLayout()

		# Titulo
		self.exLabel = QLabel("Secuenciador")
		self.exLabel.setAlignment(Qt.AlignCenter)
		self.exLabel.setStyleSheet("background-color: #00007f; font-weight: bold; color: white; border-radius: 10px;")
		self.exLabel.setMaximumHeight(30)

		self.loadActionsBtn = QPushButton("Cargar acciones del robot")
		self.loadActionsBtn.setStyleSheet("background-color: #3f3f3f")
		self.loadActionsBtn.clicked.connect(self.loadActionsClicked)

		# Layout de selección de rutina
		self.chooseRoutCombo = QComboBox()

		# Intentar poner limites a eso
		self.argLabel 	     = QLabel("Arg")
		self.chooseAttrLine  = QLineEdit()
		self.chooseAttrLine.setValidator(QIntValidator(-32767, 32767, self))


		# Botón para agregar rutina
		self.addRoutineBtn = QPushButton("Agregar acción")
		self.addRoutineBtn.clicked.connect(self.addRoutineBtnClicked)

		# Cuadro para decidir donde insertar las acciones + un label indicandolo
		self.insertLabel = QLabel("Insertar en posición ")
		self.insertLine  = QLineEdit()
		self.insertLine.setValidator(QIntValidator(0, 100, self))

		# ELIMINAR UNA RUTINA: QLabel + QLineEdit + QPushButton
		self.delRoutLabel  = QLabel("Accion a eliminar: ")
		self.delRoutLine   = QLineEdit()
		self.delRoutButton = QPushButton("Borrar")
		self.delRoutButton.setStyleSheet("background-color: #ff3f3f")
		self.delRoutButton.clicked.connect(self.delRoutBtnClicked)

		# Tabla de comandos para generar rutina
		self.routTable = QTableWidget()
		self.routTable.setFixedHeight(400)
		self.index = 0
		self.routTable.setRowCount(0)
		self.routTable.setColumnCount(3)
		self.routTable.setHorizontalHeaderLabels(('Acción','Arg', 'Device'))
		self.routTable.setColumnWidth(0, 334)
		self.routTable.setColumnWidth(1, 120)

		# Carar ultima rutina
		self.loadRoutineBtn = QPushButton("Cargar ultima secuencia")
		self.loadRoutineBtn.setStyleSheet("background-color: #3f3fff")
		self.loadRoutineBtn.clicked.connect(self.loadRoutineBtnClicked)

		# Guardar rutina
		self.saveRoutineBtn = QPushButton("Guardar secuencia")
		self.saveRoutineBtn.setStyleSheet("background-color: #3fff3f")
		self.saveRoutineBtn.clicked.connect(self.generateBtnClicked)

		# TODO UN LUGAR DONDE HACER PRINTS PARA EL USUARIO
		self.logText = QTextEdit()
		self.logText.setReadOnly(1)



		# TODO Agregar acciones
		#self.saveActionBtn = QPushButton("Guardar acción ")
		#self.saveActionBtn.setStyleSheet("background-color: #3f3fff")
		#self.saveActionBtn.clicked.connect(self.idBtnClicked)



		# Conformación de la barra horizontal de selección de rutina
		self.addRoutineLayout.addWidget(self.chooseRoutCombo)
		self.addRoutineLayout.addWidget(self.argLabel)
		self.addRoutineLayout.addWidget(self.chooseAttrLine)
		self.addRoutineLayout.addWidget(self.insertLabel)
		self.addRoutineLayout.addWidget(self.insertLine)
		self.addRoutineLayout.addWidget(self.addRoutineBtn)

		self.removeRoutineLayout.addWidget(self.delRoutLabel)
		self.removeRoutineLayout.addWidget(self.delRoutLine)
		self.removeRoutineLayout.addWidget(self.delRoutButton)

		self.outpoutLayout.addWidget(self.loadRoutineBtn)
		self.outpoutLayout.addWidget(self.saveRoutineBtn)


		self.SecLayout.addWidget(self.exLabel)
		self.SecLayout.addWidget(self.loadActionsBtn)
		self.SecLayout.addLayout(self.addRoutineLayout)
		self.SecLayout.addLayout(self.removeRoutineLayout)
		self.SecLayout.addWidget(self.routTable)
		self.SecLayout.addWidget(self.logText)
		self.SecLayout.addLayout(self.outpoutLayout)
		#self.SecLayout.addWidget(self.saveAsActionLayout)

		self.setLayout(self.SecLayout)
		self.cnt = 0

	def idBtnClicked(self):
		self.cnt += 1
		self.logText.clear()
		self.logText.append(f"Buenas {self.cnt}")
		return

	def logToUser(self,msg):
		self.logText.clear()
		self.logText.append(msg)
		return


	def loadActionsClicked(self):
		# Store the yamls actions
		try:
			self.loadedActions = load_yaml_actions("actions.yaml")
		except:
			log_msg = "No se encuentran acciones, recuerde que es necesario que exista el fichero actions.yaml en la carpeta del ejecutable"
			self.logToUser(log_msg)
			return
		
		self.reversedDict = {}

		# Add the actions to the selection layout
		self.chooseRoutCombo.clear()
		for key in self.loadedActions.keys():
			self.chooseRoutCombo.addItem(key)
			self.reversedDict[self.loadedActions[key]["id_ros"]] = key
		
		# Log the user
		log_msg = "Acciones cargadas: \n"
		for key in self.loadedActions.keys():
			log_msg += f"{key}: {self.loadedActions[key]} \n"
		self.logToUser(log_msg)
		return


	def generateBtnClicked(self):
		routines = []
		log_msg = "Rutina almacenada "
		for i in range(0, self.index):
			itemRout = self.routTable.item(i, 0)
			attrRout = self.routTable.item(i, 1)
			routines += [self.loadedActions[itemRout.text()]["device"] ,self.loadedActions[itemRout.text()]["id_ros"] , int(attrRout.text())]
		dump_yaml_routine("sequence_generated.yaml", routines)
		log_msg += f"{routines}"
		self.logToUser(log_msg)

	def loadRoutineBtnClicked(self):
		try:
			raw_routine = load_yaml_routine("sequence_generated.yaml")
		except:
			log_msg = "No se encuentra la última secuencia generada o ha ocurrido un error, recuerde que es necesario que exista el fichero sequence_generated.yaml en la carpeta del ejecutable"
			self.logToUser(log_msg)
			return
		self.routTable.clear()
		self.index = 0
		if not self.reversedDict.keys():
			log_msg = "Error al cargar rutinas. No hay acciones cargadas. Recuerde darle al boton cargar acciones"
			self.logToUser(log_msg)
			return
		for routine in raw_routine:
			device,action,arg = routine
			self.routTable.setRowCount(self.index + 1)
			self.routTable.setItem(self.index, 0, QTableWidgetItem(self.reversedDict[action]))
			self.routTable.setItem(self.index, 1, QTableWidgetItem(str(arg)))
			self.routTable.setItem(self.index, 2, QTableWidgetItem(device))
			self.index += 1
		return

	def clearBtnClicked(self):
		self.routinesCombo.setCurrentIndex(0)

	def addRoutineBtnClicked(self):
		line = self.insertLine.text()
		if line == "": 
			line = self.index
		else:	
			line = int(line) - 1
			if line > self.index or line < 0:
				log_msg = "Error al insertar rutina. Indice invalido"
				self.logToUser(log_msg)
				return
			self.routTable.insertRow(line)

		action = self.chooseRoutCombo.currentText()
		arg    = self.chooseAttrLine.text()
		try:
			arg = int(arg)
		except:
			log_msg = "El argumento es invalido, es necesario que haya un argumento numérico"
			self.logToUser(log_msg)
			return
		if self.loadedActions[action]["min_arg"] > arg:
			log_msg = "Argumento por debajo del minimo"
			self.logToUser(log_msg)
			return
		elif self.loadedActions[action]["max_arg"] < arg:
			log_msg = "Argumento por encima del maximo"
			self.logToUser(log_msg)
			return

		self.routTable.setRowCount(self.index + 1)
		self.routTable.setItem(line, 0, QTableWidgetItem(action))
		self.routTable.setItem(line, 1, QTableWidgetItem(str(arg)))
		self.routTable.setItem(line, 2, QTableWidgetItem(self.loadedActions[action]["device"]))
		self.index += 1

		log_msg = f"Rutina {str(self.chooseRoutCombo.currentText())} con argumento {self.chooseAttrLine.text()} añadida en la posición {self.index}"
		self.logToUser(log_msg)

	def delRoutBtnClicked(self):
		row = self.delRoutLine.text()
		if row == "":
			row = self.index
		else:
			try:
				row = int(row)
			except:
				log_msg = "Que columna desea borrar"
				self.logToUser(log_msg)
				return
		if row < 0 or row > self.index:
			log_msg = "No puedo borrar ese elemento"
			self.logToUser(log_msg)
			return		

		self.routTable.removeRow(row-1)
		self.index -= 1
		return