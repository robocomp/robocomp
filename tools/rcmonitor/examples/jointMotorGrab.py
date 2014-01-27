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

		try:
			self.motors = self.proxy.getAllMotorParams()
		except Ice.Exception:
			treceback.print_exc()
		print 'Motors: '
		
		self.enableButton = QPushButton("Enable motors", self)
		self.enableButton.show()
		self.enableButton.move(0,10)
		self.connect(self.enableButton, SIGNAL('clicked()'), self.enable );
		self.enableButton.resize(200, 50)

		self.disableButton = QPushButton("Disable motors", self)
		self.disableButton.show()
		self.disableButton.move(0,self.enableButton.y()+self.enableButton.height())
		self.connect(self.disableButton, SIGNAL('clicked()'), self.disable );
		self.disableButton.resize(200, 50)

		self.label = QLabel("Grab period (ms)",self)
		self.label.show()
		self.label.move(self.disableButton.x()+self.disableButton.width(),self.disableButton.y())
		self.label.resize(200,50)

		self.ledit = QLineEdit(self)
		self.ledit.show()
		self.ledit.move(0,self.disableButton.y()+self.disableButton.height())
		self.ledit.resize(200, 40)

		self.periodSB = QSpinBox(self)
		self.periodSB.setRange(100,10000)
		self.periodSB.setValue(1000)
		self.periodSB.show()
		self.periodSB.move(self.ledit.x()+self.ledit.width(),self.ledit.y())
		self.periodSB.resize(200,40)

		self.grabButton = QPushButton("Grab positions", self)
		self.grabButton.setCheckable(True)
		self.connect(self.grabButton, SIGNAL('clicked()'), self.grab );
		self.grabButton.show()
		self.grabButton.move(0,self.ledit.y()+self.ledit.height())
		self.grabButton.resize(200, 50)

		self.grabTButton = QPushButton("Grab temperature", self)
		self.grabTButton.setCheckable(True)
		self.connect(self.grabTButton, SIGNAL('clicked()'), self.grabT );
		self.grabTButton.show()
		self.grabTButton.move(self.grabButton.x()+self.grabButton.width(),self.ledit.y()+self.ledit.height())
		self.grabTButton.resize(200, 50)

		self.setMinimumSize(210, 180)
		self.values = [None]*len(self.motors)
		
		self.time = QTime()
		self.time.start()
		
		self.job()

	def job(self):
		try:
			self.states = self.proxy.getAllMotorState()
		except Ice.Exception:
			treceback.print_exc()
		#period
		if(self.time.elapsed() > self.periodSB.value()):
			self.time.restart()
			if(self.grabButton.isChecked()):
				for name in self.states.keys():
					self.values[self.nameId[name]] = self.states.get(name).pos
				for value in self.values:
					self.out << "%.2f" % value <<" "
				self.out << "\n"
				
			if(self.grabTButton.isChecked()):
				for name in self.states.keys():
					self.values[self.nameId[name]] = self.states.get(name).temperature
				for value in self.values:
					self.out << "%.2f" % value <<" "
				self.out << str(QTime.currentTime().toString("hh:mm:ss:zzz"))<< "\n"

	def enable(self):
		try:
			self.proxy.enableBrakeAllMotors()
		except Ice.Exception:
			treceback.print_exc()
	def disable(self):
		try:
			self.proxy.releaseBrakeAllMotors()
		except Ice.Exception:
			treceback.print_exc()

	def grab(self):
		if(self.grabButton.isChecked()):
			if(self.ledit.text() == ""):
				self.QMDialog = QMessageBox(QMessageBox.Warning,"Error","Insert a file name",QMessageBox.Ok)
				self.QMDialog.exec_()
				self.grabButton.setChecked(False)
			else:
				self.file = QFile(self.ledit.text())
				self.file.open(QIODevice.Text | QIODevice.WriteOnly)
				self.out = QTextStream(self.file)
				self.nameId = {}
				i = 0;
				for item in self.motors:
					self.nameId[item.name] = i
					self.out << item.name << " "
					i = i +1
				print self.nameId
				self.out << "\n"
		else:
			self.file.close()

	def grabT(self):
		if(self.grabTButton.isChecked()):
			if(self.ledit.text() == ""):
				self.QMDialog = QMessageBox(QMessageBox.Warning,"Error","Insert a file name",QMessageBox.Ok)
				self.QMDialog.exec_()
				self.grabButton.setChecked(False)
			else:
				self.file = QFile(self.ledit.text())
				self.file.open(QIODevice.Text | QIODevice.WriteOnly)
				self.out = QTextStream(self.file)
				self.nameId = {}
				i = 0;
				for item in self.motors:
					self.nameId[item.name] = i
					self.out << item.name << " "
					i = i +1
				print self.nameId
				self.out << "\n"
		else:
			self.file.close()
