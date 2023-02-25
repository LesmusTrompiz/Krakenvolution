#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets        	import *
from .widgets.simulation_tab    import SimuladorTab
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QTimer
from rclpy.action import ActionClient
from uahrk_navigation_msgs.action import GoToPose, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from uahrk_navigation_msgs.srv import SetPose2d
from tf_transformations import euler_from_quaternion
from math import pi


WIDTH  = 1024
HEIGHT = 600

##########################
# -- CLASE APLICACIÓN -- #
##########################

# Creating the main window 
class Application(Node,	QMainWindow): 
	def __init__(self): 


		# ROS things:
		Node.__init__(self,'Rviz_interface')
		self.set_pose_client = self.create_client(SetPose2d, "set_pose")
		self.nav_client 		  = ActionClient(self, Path, 'move_server')
		self.path_finding_client  = ActionClient(self, GoToPose, 'path_finding_server')
		self.pub_obstacles = self.create_publisher(PoseArray, 'obstacles', 10)
		self.obstacles = PoseArray()
		self.sub_init_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.cb_update_pose,
            10)

		self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.cb_new_goal,
            10)

		# QT things
		QMainWindow.__init__(self)
		self.setWindowTitle('UAHR Krakens Team - Eurobot 2023') 
		self.setGeometry(0, 0, WIDTH, HEIGHT) 
		modes = [("Path Finding", "path_finding"), ("Navigation", "navigation"), ("Grid","grid")]
		self.tab_manager = TabManager(self, modes) 
		self.setCentralWidget(self.tab_manager) 
		self.show()

		# Start QT timer
		# creating a timer object
		timer = QTimer(self)
        # adding action to timer
		timer.timeout.connect(self.timer_callback)
 
        # update the timer every second
		timer.start(10)



	def listener_cb(self, msg):
		self.get_logger().info('I heard: "%s"' % msg.data)
		data = String()
		data.data = "a"
		self.publisher_.publish(data)
	
	def cb_update_pose(self, pose : PoseWithCovarianceStamped):
		orientation_list = [pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w]
		(_, _, yaw) = euler_from_quaternion(orientation_list)
		self.get_logger().info('Recibido: ')
		new_pose = SetPose2d.Request()
		new_pose.x = pose.pose.pose.position.x
		new_pose.y = pose.pose.pose.position.y
		new_pose.a = yaw * 180 / pi
		self.get_logger().info('Pidiendo: ')

		self.set_pose_client.call_async(new_pose)
		self.get_logger().info('Acabao: ')

	def cb_new_goal(self, new_goal : PoseStamped):

		mode = self.tab_manager.get_mode()
		self.get_logger().info(f'Modo: {mode}')

		if mode == "navigation":
			self.get_logger().info(f'Navigation: {mode}')
			goal_msg = Path.Goal()
			goal_msg.pose.poses.append(new_goal.pose)
			self.nav_client.send_goal_async(goal_msg)
			self.get_logger().info(f'Navigation GOal: {mode}')

		elif mode ==  "path_finding":
			self.get_logger().info('Path finding goal : ')
			goal_msg = GoToPose.Goal()
			goal_msg._pose = new_goal
			self.path_finding_client.send_goal_async(goal_msg)
		elif mode == "grid":
			self.obstacles.poses += [new_goal.pose]


			


	def timer_callback(self):
		if self.tab_manager.get_clear_state():
			self.obstacles = PoseArray()
		self.pub_obstacles.publish(self.obstacles)
		rclpy.spin_once(self,timeout_sec=0)

############################
# -- GESTOR DE PESTAÑAS -- #
############################

# Creating tab widget
class TabManager(QWidget): 
	def __init__(self, parent, modes): 
		super(QWidget, self).__init__(parent) 
		self.layout = QVBoxLayout(self) 

		# Initialize tab screen 
		self.tabM = QTabWidget()
		self.tabM.resize(WIDTH, HEIGHT) 

		# Add tabs, choose icon and name
		#self.tabM.addTab(HomeTab(), QIcon("img/boat.png"),("Inicio")) 
		#self.tabM.addTab(ParamTab(), QIcon("img/compass.png"), ("Parámetros"))
		#self.tabM.addTab(PointsTab(), QIcon("img/crab.png"), ("Puntuación")) 
		self.sim_tab = SimuladorTab(modes)

		self.tabM.addTab(self.sim_tab, "Simulador") 

		# Add tabs to widget 
		self.layout.addWidget(self.tabM) 
		self.setLayout(self.layout)

	def get_mode(self):
		return self.sim_tab.get_mode()
	
	def get_clear_state(self):
		return self.sim_tab.get_clear_state()

#######################
# -- MAIN FUNCTION -- #
#######################

#if __name__ == '__main__': 
def main():
	rclpy.init()
	app  = QApplication(sys.argv) 
	root = Application() 
	print("main")
	sys.exit(app.exec_()) 