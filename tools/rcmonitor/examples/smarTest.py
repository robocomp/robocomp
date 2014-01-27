# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import Ice, sys, math, traceback, random, copy,time, decimal

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *
from ui_smarTestDlg import Ui_windowSmarTest

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompSmar'].SmarPrx.checkedCast(self.prx)
		self.ui = Ui_windowSmarTest()
		self.ui.setupUi(self)
		self.show()
		
		print "AQUIIIIIIIIIIIIIIIIIIII"
		
		#~ print float('inf')
		#~ #TEST ZONE
		#~ self.testWidget = QGroupBox(self)
		#~ self.testWidget.resize(250, 240)
		#~ self.testWidget.move(0, 0)
		#~ self.testWidget.setTitle('Smar Test Panel')
		#~ self.testWidget.show()
		#~ 
		#~ 
		#~ self.turnRight90Button = QPushButton("Turn right 90", self)
		#~ self.turnRight90Button.show()
		#~ self.turnRight90Button.move(0, 20)
		#~ self.connect(self.turnRight90Button, SIGNAL('clicked()'), self.right90Clicked)
		#~ self.turnRight90Button.resize(250, 40)
		#~ 
		#~ 
		#~ self.turnRadiansButton = QPushButton("Turn radians...", self)
		#~ self.turnRadiansButton.show()
		#~ self.turnRadiansButton.move(0,self.turnRight90Button.y()+self.turnRight90Button.height())
		#~ self.connect(self.turnRadiansButton, SIGNAL('clicked()'), self.turnRadiansClicked)
		#~ self.turnRadiansButton.resize(250, 40)
		#~ 
		#~ self.angleLabel = QLabel(self)
		#~ self.angleLabel.setText("Angle (radians):")
		#~ self.angleLabel.resize(120, 70)
		#~ self.angleLabel.move(self.turnRight90Button.x()+self.turnRight90Button.width(),self.turnRight90Button.y())
		#~ self.angleLabel.show()
		#~ self.angleSpinbox = QDoubleSpinBox(self)
		#~ self.angleSpinbox.show()
		#~ self.angleSpinbox.move(self.turnRadiansButton.x()+self.turnRadiansButton.width(),self.turnRadiansButton.y())
		#~ self.angleSpinbox.resize(250, 40)
		#~ self.angleSpinbox.setMaximum(3.14)
		#~ self.angleSpinbox.setMinimum(-3.14)
		#~ self.angleSpinbox.setSingleStep(0.1)
		#~ 
		#~ 
		#~ 
		#~ 
		#~ 
		#~ 
		#~ self.rightSquareButton = QPushButton("Right Square", self)
		#~ self.rightSquareButton.show()
		#~ self.rightSquareButton.move(0,self.turnRadiansButton.y()+self.turnRadiansButton.height())
		#~ self.connect(self.rightSquareButton, SIGNAL('clicked()'), self.rightSquareClicked)
		#~ self.rightSquareButton.resize(250, 40)
		#~ 
		#~ self.leftSquareButton = QPushButton("Left Square", self)
		#~ self.leftSquareButton.show()
		#~ self.leftSquareButton.move(0,self.rightSquareButton.y()+self.rightSquareButton.height())
		#~ self.connect(self.leftSquareButton, SIGNAL('clicked()'), self.leftSquareClicked)
		#~ self.leftSquareButton.resize(250, 40)
		#~ 
		#~ 
		#~ self.distanceLabel = QLabel(self)
		#~ self.distanceLabel.setText("Distance (mm):")
		#~ self.distanceLabel.resize(120, 70)
		#~ self.distanceLabel.move(self.rightSquareButton.x()+self.rightSquareButton.width(),self.rightSquareButton.y())
		#~ self.distanceLabel.show()
		#~ self.distanceSpinbox = QDoubleSpinBox(self)
		#~ self.distanceSpinbox.show()
		#~ self.distanceSpinbox.move(self.leftSquareButton.x()+self.leftSquareButton.width(),self.leftSquareButton.y())
		#~ self.distanceSpinbox.resize(250, 40)
		#~ self.distanceSpinbox.setMaximum(5000.)
		#~ self.distanceSpinbox.setMinimum(-5000.)
		#~ self.distanceSpinbox.setSingleStep(10)

		#~ self.advanceButton = QPushButton("Advance...", self)
		#~ self.advanceButton.show()
		#~ self.advanceButton.move(0,self.leftSquareButton.y()+self.leftSquareButton.height())
		#~ self.connect(self.advanceButton, SIGNAL('clicked()'), self.moveForward)
		#~ self.advanceButton.resize(250, 40)
		
		self.connect(self.ui.advancebutton, SIGNAL('clicked()'), self.moveForward)
		self.connect(self.ui.turn90rightButton, SIGNAL('clicked()'), self.right90Clicked)
		self.connect(self.ui.turn90leftButton, SIGNAL('clicked()'), self.left90Clicked)
		self.connect(self.ui.turnRadiansButton, SIGNAL('clicked()'), self.turnRadiansClicked)
		self.connect(self.ui.rightSquareButton, SIGNAL('clicked()'), self.rightSquareClicked)
		self.connect(self.ui.leftSquareButton, SIGNAL('clicked()'), self.leftSquareClicked)
		
		self.job()
	


	def job(self):
		self.current_state = self.mods['RoboCompSmar'].TBaseState()
		self.current_state = self.proxy.getBaseState()
		self.ui.SmarPositionLabel.setText(QString("X: "+str(self.current_state.x)+"    Y: "+str(self.current_state.z)+"    Yaw: "+str(self.current_state.alpha)))
		#~ print "job"
		#while(True):
			#self.proxy.setPosition(goal1)
			#time.sleep(6)
			#self.proxy.setPosition(goal2)
			#time.sleep(6)
		#~ try:
			#~ self.states = self.proxy.getAllMotorState()
			#~ self.states = 0
			#~ self.refused = 0
			#~ state = self.states[str(self.combo.currentText())]
			#~ self.readLabel.setText(QString.number(state.pos))
		#~ except Ice.ConnectionRefusedException:
			#~ if self.refused == 0:
				#~ self.refused = 1
				#~ print 'Connection refused'
			#~ elif self.refused == 1:
				#~ self.refused == 2
		#self.clicked()

	#~ TODO: CONTROLAR EXCEPCION DE GIRO SOBRE 0
	def right90Clicked(self):
		print "right90Clicked"
		print "with radius of",self.ui.displaceVAlue.value(),"m and Velocity:",self.ui.speedValue.value()
		#~ worker->computeAPJE(radius, vel);
		#get base state
		self.old_state = self.mods['RoboCompSmar'].TBaseState()
		self.new_state = self.mods['RoboCompSmar'].TBaseState()
		self.old_state = self.proxy.getBaseState() 
		
		
		radius = math.fabs(self.ui.radiusValue.value())
		base_vel = self.ui.speedValue.value()
		self.proxy.setSpeedBase(0.0, radius);
		#turn right
		time.sleep(3)
		self.proxy.setSpeedBase(base_vel, radius);
		turning = True;
		reduced = False;
		#Wait until 90 turned
		while(turning):
			self.new_state = self.proxy.getBaseState()
			#~ print "old"+str(self.old_state.alpha)
			#~ print "new"+str(self.new_state.alpha)
			distance = math.fabs(self.new_state.alpha-self.old_state.alpha)
			if(distance>=(math.pi/2)):
				turning = False
				print math.fabs(self.new_state.alpha-self.old_state.alpha)
			elif(distance>=1.2):
				d = math.fabs(math.pi/2. - distance)
				print d
				new_velocity = self.calculateVelocity(base_vel, math.fabs(1.2-math.pi/2.), d)
				print "new velocity ", new_velocity
				if(new_velocity>0.02):
					self.proxy.setSpeedBase(new_velocity, radius)		
			
			time.sleep(0.1)
		self.proxy.setSpeedBase(0.0, radius);
		time.sleep(3)
		self.proxy.setSpeedBase(0.0, float('inf'))
		
		
	#~ TODO: CONTROLAR EXCEPCION DE GIRO SOBRE 0
	def left90Clicked(self):
		print "left90Clicked"
		print "with radius of",self.ui.radiusValue.value(),"m and Velocity:",self.ui.speedValue.value()
		self.old_state = self.mods['RoboCompSmar'].TBaseState()
		self.new_state = self.mods['RoboCompSmar'].TBaseState()
		self.old_state = self.proxy.getBaseState() 
		radius = -math.fabs(self.ui.radiusValue.value())
		base_vel = self.ui.speedValue.value()
		if radius == 0:
			base_vel = -math.fabs(base_vel)
		self.proxy.setSpeedBase(0.0, -math.fabs(radius));
		#turn right
		time.sleep(3)
		self.proxy.setSpeedBase(base_vel, -math.fabs(radius));
		turning = True;
		reduced = False;
		#Wait until 90 turned
		while(turning):
			self.new_state = self.proxy.getBaseState()
			#~ print "old"+str(self.old_state.alpha)
			#~ print "new"+str(self.new_state.alpha)
			distance = math.fabs(self.new_state.alpha-self.old_state.alpha)
			if(distance>=(math.pi/2)):
				turning = False
				print math.fabs(self.new_state.alpha-self.old_state.alpha)
			elif(distance>=1.2):
				d = math.fabs(math.pi/2. - distance)
				print d
				new_velocity = self.calculateVelocity(base_vel, math.fabs(1.2-math.pi/2.), d)
				if radius == 0:
					new_velocity = -math.fabs(new_velocity)
				print "new velocity ", new_velocity
				if(new_velocity>0.02):
					self.proxy.setSpeedBase(new_velocity, -math.fabs(radius))		
			time.sleep(0.1)
		self.proxy.setSpeedBase(0.0, -math.fabs(radius));
		time.sleep(3)
		self.proxy.setSpeedBase(0.0, float('inf'))
		
				
	def calculateVelocity(self, base_vel, cut_pos, distance):
#		print str(base_vel)+" - (("+str(base_vel)+"/"+str(cut_pos)+")*("+str(distance)+"-"+str(cut_pos)+"))"
#		return base_vel - ((base_vel/cut_pos)*(distance-cut_pos))
#					f = satura(K*disancia + C, Vm)
		vel = (base_vel/cut_pos) * math.fabs(distance) + 0.02
		if distance < 0: vel = -vel
		if math.fabs(vel) > math.fabs(base_vel):
			vel = base_vel
		return vel
		
	def waitRadians(self, radians):
		pass
		
	def waitAdvance(self, milimeters):
		
		pass
		
	def moveForward(self):
		print "moveForward"
		print "with avance of",self.ui.displaceVAlue.value(),"mm and velocity",self.ui.speedValue.value()
		self.old_state = self.mods['RoboCompSmar'].TBaseState()
		self.new_state = self.mods['RoboCompSmar'].TBaseState()
		self.old_state = self.proxy.getBaseState()
		base_vel = self.ui.speedValue.value()
		goal_dist = self.ui.displaceVAlue.value()
		if goal_dist > 0:
			self.proxy.setSpeedBase(base_vel, float('inf'))
		else:
			self.proxy.setSpeedBase(base_vel, float('-inf')); 
		advancing = True;
		while(advancing):
			self.new_state = self.proxy.getBaseState()
			xx=self.new_state.x-self.old_state.x
			yy=self.new_state.z-self.old_state.z
			actual_distance = math.sqrt(math.pow(xx,2)+math.pow(yy,2))
			print "Actual distance: "+str(actual_distance), self.new_state.alpha - self.old_state.alpha
			if(actual_distance>=math.fabs(goal_dist)):
				advancing = False
				self.proxy.setSpeedBase(0.0, float('inf'))
			else:
				base_vel2 = base_vel
				if math.fabs(actual_distance-math.fabs(goal_dist)) < 1000.:
					base_vel2 = 0.3
				elif math.fabs(actual_distance-math.fabs(goal_dist)) < 500.:
					base_vel2 = 0.2
				if math.fabs(actual_distance-math.fabs(goal_dist)) < 300.:
					base_vel2 = 0.1
				if goal_dist > 0:
					self.proxy.setSpeedBase(base_vel2, float('inf'))
				else:
					self.proxy.setSpeedBase(base_vel2, float('-inf')); 

			time.sleep(0.1)
			
		
	def rightSquareClicked(self):
		print "rightSquareClicked"
		print "with radius",self.ui.radiusValue.value(),"and velocity",self.ui.speedValue.value()
		self.moveForward()
		self.right90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.right90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.right90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.right90Clicked()
		
		
	def leftSquareClicked(self):
		print "leftSquareClicked"
		print "with radius",self.ui.radiusValue.value(),"and velocity",self.ui.speedValue.value()
		self.moveForward()
		self.left90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.left90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.left90Clicked()
		time.sleep(1.8)
		self.moveForward()
		self.left90Clicked()
	
	#TODO: COMPROBAR QUE LA VELOCIDAD NO ES 0
	def turnRadiansClicked(self):
		#~ print "NOT IMPLEMENTE YET"
		print "turnRadiansClicked"
		print self.ui.angDisplaceValue.value(),"radians with radius",self.ui.radiusValue.value(),"and velocity",self.ui.speedValue.value()
		self.old_state = self.mods['RoboCompSmar'].TBaseState()
		self.new_state = self.mods['RoboCompSmar'].TBaseState()
		
		self.old_state = self.proxy.getBaseState() 
		
		radius = self.ui.radiusValue.value()
		base_vel = self.ui.speedValue.value()
		radians = self.ui.angDisplaceValue.value()
		self.proxy.setSpeedBase(0.0,radius)
		#turn right
		time.sleep(3)
		self.proxy.setSpeedBase(base_vel,radius);
		turning = True;
		reduced = False;
		#Wait until 90 turned
		while(turning):
			self.new_state = self.proxy.getBaseState()
			distance = math.fabs(self.new_state.alpha-self.old_state.alpha)
			d = math.fabs(radians - distance)
			print "Actual alpha: ", self.new_state.alpha - self.old_state.alpha
			if(distance>=radians):
				turning = False
				print math.fabs(self.new_state.alpha-self.old_state.alpha)
			elif(d<0.3):
				print d
				new_velocity = self.calculateVelocity(base_vel, 0.3, d)
				print "new velocity ", new_velocity
				if(new_velocity>0.02):
					self.proxy.setSpeedBase(new_velocity,radius)		
			time.sleep(0.1)
		self.proxy.setSpeedBase(0.0,radius);
		time.sleep(3)
		self.proxy.setSpeedBase(0.0, float('inf'))
		
