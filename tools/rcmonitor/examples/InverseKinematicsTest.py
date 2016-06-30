## -*- coding: utf-8 -*-

#    Copyright (C) 2016 by RoboLab - University of Extremadura
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
import random, time
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *

#Position dictionary:{'name':(,x,y,z,rx,ry,rz),...}
posDict = {'zero':(0,0,800,0,0,0)}
#Body part to send commands:
bodyPart = "ARM"


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompInverseKinematics'].InverseKinematicsPrx.checkedCast(self.prx)
		self.show()

		self.combo = QComboBox(self)
		self.combo.show()
		self.combo.resize(200, 50)
		
		print 'Load positions: ',
		for name in posDict.keys():
			print name
			self.combo.addItem(name)
	
		self.setPosition = QPushButton("Go pos List", self)
		self.setPosition.show()
		self.setPosition.move(self.combo.x()+self.combo.width(),0)
		self.connect(self.setPosition, SIGNAL('clicked()'), self.set_position_list );
		self.setPosition.resize(200, 50)

		self.label1 = QLabel('X, Y, Z',self)
		self.label1.show()
		self.label1.move(0,self.combo.y()+self.combo.height())
		self.label1.resize(200, 50)
		
		self.x_sb = QSpinBox(self)
		self.x_sb.show()
		self.x_sb.move(0,self.label1.y()+self.label1.height())
		self.x_sb.resize(200, 50)
		self.x_sb.setMaximum(1000)
		self.x_sb.setMinimum(0)
		self.x_sb.setSingleStep(10)
		
		self.y_sb = QSpinBox(self)
		self.y_sb.show()
		self.y_sb.move(0,self.x_sb.y()+self.x_sb.height())
		self.y_sb.resize(200, 50)
		self.y_sb.setMaximum(1000)
		self.y_sb.setMinimum(0)
		self.y_sb.setSingleStep(10)
		
		self.z_sb = QSpinBox(self)
		self.z_sb.show()
		self.z_sb.move(0,self.y_sb.y()+self.y_sb.height())
		self.z_sb.resize(200, 50)
		self.z_sb.setMaximum(1000)
		self.z_sb.setMinimum(0)
		self.z_sb.setSingleStep(10)
		
		self.label2 = QLabel('RX, RY, RZ',self)
		self.label2.show()
		self.label2.move(self.label1.x()+self.label1.width(),self.setPosition.y()+self.setPosition.height())
		self.label2.resize(200, 50)
		
		self.rx_sb = QDoubleSpinBox(self)
		self.rx_sb.show()
		self.rx_sb.move(self.label2.x(),self.label2.y()+self.label2.height())
		self.rx_sb.resize(200, 50)
		self.rx_sb.setMaximum(3.14)
		self.rx_sb.setMinimum(-3.14)
		self.rx_sb.setSingleStep(0.1)
		
		self.ry_sb = QDoubleSpinBox(self)
		self.ry_sb.show()
		self.ry_sb.move(self.label2.x(),self.rx_sb.y()+self.rx_sb.height())
		self.ry_sb.resize(200, 50)
		self.ry_sb.setMaximum(3.14)
		self.ry_sb.setMinimum(-3.14)
		self.ry_sb.setSingleStep(0.1)
		
		self.rz_sb = QDoubleSpinBox(self)
		self.rz_sb.show()
		self.rz_sb.move(self.label2.x(),self.ry_sb.y()+self.ry_sb.height())
		self.rz_sb.resize(200, 50)
		self.rz_sb.setMaximum(3.14)
		self.rz_sb.setMinimum(-3.14)
		self.rz_sb.setSingleStep(0.1)

		self.goPosition = QPushButton("Go position", self)
		self.goPosition.show()
		self.goPosition.move(0,self.z_sb.y()+self.z_sb.height())
		self.connect(self.goPosition, SIGNAL('clicked()'), self.set_position );
		self.goPosition.resize(200, 50)

		self.stopButton = QPushButton("Stop", self)
		self.stopButton.show()
		self.stopButton.move(self.rz_sb.x(),self.rz_sb.y()+self.rz_sb.height())
		self.connect(self.stopButton, SIGNAL('clicked()'), self.stop );
		self.stopButton.resize(200, 50)
		

	def job(self):
		pass
#		self.states = self.proxy.getAllMotorState()
		#for k in self.states.keys():
			#print k, self.states[k].pos
		#print '-------'
#		state = self.states[str(self.combo.currentText())]
#		self.readLabel.setText(QString.number(state.pos))
#		self.templabel.setText(QString.number(state.temperature))
		#self.clicked()
#		string = "overheat(>60):   "
#		for k in self.states.keys():
#			if(self.states[k].temperature > 60):
#				string = string + k + "(" + str(self.states[k].temperature) +")   "
#		self.warninglabel.setText(string)

	def set_position(self):
		target = self.mods['RoboCompInverseKinematics'].Pose6D()
		target.x = self.x_sb.value()
		target.y = self.y_sb.value()
		target.z = self.z_sb.value()
		target.rx = self.rx_sb.value()
		target.ry = self.ry_sb.value()
		target.rz = self.rz_sb.value()
		self.sendMove(target)
		

	def set_position_list(self):
		target = self.mods['RoboCompInverseKinematics'].Pose6D()
		values = posDict[str(self.combo.currentText())]
		target.x = values[0]
		target.y = values[1]
		target.z = values[2]
		target.rx = values[3]
		target.ry = values[4]
		target.rz = values[5]
		self.sendMove(target)

	def stop(self):
		try:
			self.proxy.stop(bodyPart)
		except:
			print sys.exc_info()[0]

	def sendMove(self,target):
		weights = self.mods['RoboCompInverseKinematics'].WeightVector()
		weights.x = 1
		weights.y = 1
		weights.z = 1
		weights.rx = 1
		weights.ry = 1
		weights.rz = 1
		try:
			self.proxy.setTargetPose6D(bodyPart, target, weights)
		except:
			print sys.exc_info()[0]


