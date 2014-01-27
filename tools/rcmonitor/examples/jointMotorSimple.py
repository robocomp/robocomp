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

		self.combo = QComboBox(self)
		self.combo.show()
		self.combo.resize(200, 50)

		self.motors = self.proxy.getAllMotorParams()
		print 'Motors: ',
		for item in self.motors:
			print item.name
			self.combo.addItem(item.name)
		if len(self.motors)==0: print 'JointMotor: Error: No motors.'

		self.enableButton = QPushButton("Enable", self)
		self.enableButton.show()
		self.enableButton.move(self.combo.x()+self.combo.width(),0)
		self.connect(self.enableButton, SIGNAL('clicked()'), self.enable );
		self.enableButton.resize(200, 50)

		self.disableButton = QPushButton("Disable", self)
		self.disableButton.show()
		self.disableButton.move(self.combo.x()+self.combo.width(),self.enableButton.y()+self.enableButton.height())
		self.connect(self.disableButton, SIGNAL('clicked()'), self.disable );
		self.disableButton.resize(200, 50)

		self.stopButton = QPushButton("Stop", self)
		self.stopButton.show()
		self.stopButton.move(self.combo.x()+self.combo.width(),self.disableButton.y()+self.disableButton.height())
		self.connect(self.stopButton, SIGNAL('clicked()'), self.stop );
		self.stopButton.resize(200, 50)

		self.readLabel = QLabel(self)
		self.readLabel.show()
		self.readLabel.move(0,self.combo.y()+self.combo.height())
		self.readLabel.resize(200, 50)

		self.spinBox = QDoubleSpinBox(self)
		self.spinBox.show()
		self.spinBox.move(0,self.readLabel.y()+self.readLabel.height())
		self.spinBox.resize(200, 50)
		self.spinBox.setMaximum(1024.)
		self.spinBox.setMinimum(-200.)
		self.spinBox.setSingleStep(0.1)

		self.commandButton = QPushButton("Set position", self)
		self.commandButton.show()
		self.commandButton.move(0,self.spinBox.y()+self.spinBox.height())
		self.connect(self.commandButton, SIGNAL('clicked()'), self.clicked );
		self.commandButton.resize(200, 50)
		
		self.spinBox2 = QDoubleSpinBox(self)
		self.spinBox2.show()
		self.spinBox2.move(0,self.commandButton.y()+self.commandButton.height())
		self.spinBox2.resize(200, 50)
		self.spinBox2.setDecimals(5)
		self.spinBox2.setMaximum(6.000)
		self.spinBox2.setMinimum(-6.000)
		self.spinBox2.setSingleStep(0.001)

		self.commandButton2 = QPushButton("Set velocity", self)
		self.commandButton2.show()
		self.commandButton2.move(0,self.spinBox2.y()+self.spinBox2.height())
		self.connect(self.commandButton2, SIGNAL('clicked()'), self.clicked2 );
		self.commandButton2.resize(200, 50)

		self.label = QLabel("Temperature",self)
		self.label.show()
		self.label.move(0,self.commandButton2.y()+self.commandButton2.height())
		
		self.templabel = QLabel(self)
		self.templabel.show()
		self.templabel.move(0,self.label.y()+self.label.height())
		self.templabel.resize(200,50)

		self.warninglabel = QLabel(self)
		self.warninglabel.show()
		self.warninglabel.move(0,self.templabel.y()+self.templabel.height())
		self.warninglabel.resize(200,50)

		self.setMinimumSize(210, 180)
		self.states = self.proxy.getAllMotorState()
		self.job()

	def job(self):
		self.states = self.proxy.getAllMotorState()
		#for k in self.states.keys():
			#print k, self.states[k].pos
		#print '-------'
		state = self.states[str(self.combo.currentText())]
		self.readLabel.setText(QString.number(state.pos))
		self.templabel.setText(QString.number(state.temperature))
		#self.clicked()
		string = "overheat(>60):   "
		for k in self.states.keys():
			if(self.states[k].temperature > 60):
				string = string + k + "(" + str(self.states[k].temperature) +")   "
		self.warninglabel.setText(string)

	def clicked(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
		goal.position = self.spinBox.value()
		goal.name = str(self.combo.currentText())
		goal.maxSpeed = 0.5
		print "target pos" , goal.position, "name", goal.name, "maxSpeed" , goal.maxSpeed
		self.proxy.setPosition(goal)
	def clicked2(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
		goal.velocity = self.spinBox2.value()
		goal.name = str(self.combo.currentText())
		goal.maxAcc = 1.5
		print goal.velocity, goal.name, goal.maxAcc
		self.proxy.setReferenceVelocity(goal)
	def enable(self):
		self.proxy.enableBrakeMotor(str(self.combo.currentText()))
	def disable(self):
		self.proxy.releaseBrakeMotor(str(self.combo.currentText()))
	def stop(self):
		self.proxy.stopMotor(str(self.combo.currentText()))
