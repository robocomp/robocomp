## -*- coding: utf-8 -*-

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

import Ice, sys, math, traceback

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompJointMotor'].JointMotorPrx.checkedCast(self.prx)
		self.show()
		self.layout = QGridLayout(self)

		self.addMotorParams()
		self.addPositionParams()
		self.addVelocityParams()
		self.addLabels()

		self.setMinimumSize(210, 180)
		self.states = self.proxy.getAllMotorState()
		self.job()

	def addMotorParams(self):
		# Motor list
		self.combo = QComboBox(self)
		#self.combo.resize(200, 50)
		self.layout.addWidget( self.combo, 0, 0 )
		self.motors = self.proxy.getAllMotorParams()
		for item in self.motors:
			self.combo.addItem(item.name)
		if len(self.motors)==0:
			print 'JointMotor: Error: No motors.'
		
		# Enable motor
		self.enableButton = QPushButton("Enable", self)
		self.layout.addWidget( self.enableButton, 0, 1 )
		self.connect(self.enableButton, SIGNAL('clickedPosition()'), self.enable )

		# Disable motor
		self.disableButton = QPushButton("Disable", self)
		self.layout.addWidget( self.disableButton, 0, 2 )
		self.connect(self.disableButton, SIGNAL('clickedPosition()'), self.disable )

		# Stop motor
		self.stopButton = QPushButton("Stop", self)
		self.layout.addWidget( self.stopButton, 0, 3 )
		self.connect(self.stopButton, SIGNAL('clickedPosition()'), self.stop )
	
	# Set position
	def addPositionParams(self):
		# Label
		self.labelPosition = QLabel("Target position",self)
		self.layout.addWidget( self.labelPosition, 1, 0 )
		
		# Target position
		self.spinPosition1 = QDoubleSpinBox(self)
		self.spinPosition1.setDecimals(5)
		self.spinPosition1.setMaximum(1024.)
		self.spinPosition1.setMinimum(-200.)
		self.spinPosition1.setSingleStep(0.1)
		self.layout.addWidget( self.spinPosition1, 1, 1 )
		
		# Maximum velocity
		self.spinPosition2 = QDoubleSpinBox(self)
		self.spinPosition2.setDecimals(5)
		self.spinPosition2.setMaximum(6.000)
		self.spinPosition2.setMinimum(-6.000)
		self.spinPosition2.setSingleStep(0.001)
		self.layout.addWidget( self.spinPosition2, 1, 2 )

		# Make it so
		self.buttonPosition = QPushButton("Set position", self)
		self.layout.addWidget( self.buttonPosition, 1, 3 )
		self.connect(self.buttonPosition, SIGNAL('clicked()'), self.clickedPosition );
	
	# Set velocity
	def addVelocityParams(self):
		# Label
		self.labelVelocity = QLabel("Target velocity",self)
		self.layout.addWidget( self.labelVelocity, 2, 0 )
		
		# Target velocity
		self.spinVelocity1 = QDoubleSpinBox(self)
		self.spinVelocity1.setDecimals(5)
		self.spinVelocity1.setMaximum(6.000)
		self.spinVelocity1.setMinimum(-6.000)
		self.spinVelocity1.setSingleStep(0.001)
		self.layout.addWidget( self.spinVelocity1, 2, 1 )

		# Maximum acceleration
		self.spinVelocity2 = QDoubleSpinBox(self)
		self.spinVelocity2.setDecimals(5)
		self.spinVelocity2.setMaximum(6.000)
		self.spinVelocity2.setMinimum(-6.000)
		self.spinVelocity2.setSingleStep(0.001)
		self.layout.addWidget( self.spinVelocity2, 2, 2 )
		
		# Make it so
		self.buttonVelocity = QPushButton("Set velocity", self)
		self.layout.addWidget( self.buttonVelocity, 2, 3 )
		self.connect(self.buttonVelocity, SIGNAL('clicked()'), self.clickedPositionVelocity );
	
	# Labels
	def addLabels(self):
		# Label
		self.readLabel = QLabel(self)
		self.layout.addWidget( self.readLabel, 3, 0 )
		
		self.label = QLabel("Temperature",self)
		self.layout.addWidget( self.label, 3, 1 )
		
		self.templabel = QLabel(self)
		self.layout.addWidget( self.templabel, 3, 2 )

		self.warninglabel = QLabel(self)
		self.layout.addWidget( self.warninglabel, 3, 3 )
	
	def job(self):
		self.states = self.proxy.getAllMotorState()
		#for k in self.states.keys():
			#print k, self.states[k].pos
		#print '-------'
		state = self.states[str(self.combo.currentText())]
		self.readLabel.setText(QString.number(state.pos))
		self.templabel.setText(QString.number(state.temperature))
		#self.clickedPosition()
		string = "overheat(>60):   "
		for k in self.states.keys():
			if(self.states[k].temperature > 60):
				string = string + k + "(" + str(self.states[k].temperature) +")   "
		self.warninglabel.setText(string)

	def clickedPosition(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
		goal.name = str(self.combo.currentText())
		goal.position = self.spinPosition1.value()
		goal.maxSpeed = self.spinPosition2.value()
		self.proxy.setPosition(goal)
	
	def clickedPositionVelocity(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
		goal.name = str(self.combo.currentText())
		goal.velocity = self.spinVelocity1.value()
		goal.maxAcc = self.spinVelocity2.value()
		self.proxy.setVelocity(goal)
	
	def enable(self):
		self.proxy.enableBrakeMotor(str(self.combo.currentText()))
	
	def disable(self):
		self.proxy.releaseBrakeMotor(str(self.combo.currentText()))
	
	def stop(self):
		self.proxy.stopMotor(str(self.combo.currentText()))
