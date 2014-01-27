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
		self.proxy = self.mods['RoboCompUrsusArm'].UrsusArmPrx.checkedCast(self.prx)
		self.show()

		self.motors = []
		try:
			self.motors = self.proxy.getAllMotorState()
		except Ice.Exception:
			treceback.print_exc()
		
		self.layout = QVBoxLayout(self)
		#motors form
		self.layoutF = QVBoxLayout()
		self.ledMotors = {}
		for i in self.motors.keys():
			self.l = QHBoxLayout()
#			self.l.addSpacerItem(QSpacerItem(10,10,QSizePolicy.MinimumExpanding,QSizePolicy.Fixed))
			self.label = QLabel(i,self)
			self.l.addWidget(self.label)
			self.ledMotors[i] = QLCDNumber(self)
			self.l.addWidget(self.ledMotors[i])
			self.layoutF.addLayout(self.l)
		self.layout.addLayout(self.layoutF)

		self.rbLeft = QRadioButton("Left")
		self.rbLeft.setChecked(True)
		self.rbRight = QRadioButton("Right")
		self.bgroup = QButtonGroup(self)
		self.bgroup.addButton(self.rbLeft)
		self.bgroup.addButton(self.rbRight)
		self.bgroup.setExclusive(True)
		self.connect(self.bgroup,SIGNAL('buttonClicked(int)'),self.changeArmSlot)
		self.hlayout4 = QHBoxLayout()
		self.hlayout4.addWidget(self.rbLeft)
		self.hlayout4.addWidget(self.rbRight)
		self.layout.addLayout(self.hlayout4)

		self.lfile = QLineEdit(self)
		self.hfilelayout = QHBoxLayout()
		self.hfilelayout.addWidget(self.lfile)
		self.loadButton = QPushButton("Load",self)
		self.connect(self.loadButton,SIGNAL('clicked()'),self.loadSlot)
		self.hfilelayout.addWidget(self.loadButton)
		self.layout.addLayout(self.hfilelayout)
		
		self.hlayout2 = QHBoxLayout()
		self.reloadB = QPushButton("Reload",self)
		self.connect(self.reloadB,SIGNAL('clicked()'),self.reloadSlot)
		self.hlayout2.addWidget(self.reloadB)
		self.pauseB = QPushButton("Pause",self)
		self.pauseB.setCheckable(True)
		self.connect(self.pauseB,SIGNAL('clicked()'),self.pauseSlot)
		self.hlayout2.addWidget(self.pauseB)
		self.layout.addLayout(self.hlayout2)
		self.leftPaused = False
		self.rightPaused = False
		
		self.hlayout3 = QHBoxLayout()
		self.stopB = QPushButton("Stop",self)
		self.connect(self.stopB,SIGNAL('clicked()'),self.stopSlot)
		self.hlayout3.addWidget(self.stopB)
		self.goRestB = QPushButton("Go rest",self)
		self.connect(self.goRestB,SIGNAL('clicked()'),self.goRestSlot)
		self.hlayout3.addWidget(self.goRestB)
		self.layout.addLayout(self.hlayout3)

		self.hlayout5 = QHBoxLayout()
		self.sbSpeed = QSpinBox(self)
		self.hlayout5.addWidget(self.sbSpeed)
		self.pbSpeed = QPushButton("Speed",self)
		self.connect(self.pbSpeed,SIGNAL('clicked()'),self.speedSlot)
		self.hlayout5.addWidget(self.pbSpeed)
		self.layout.addLayout(self.hlayout5)

		self.hlayout6 = QHBoxLayout()
		self.sbRepetitions = QSpinBox(self)
		self.sbRepetitions.setRange(1,100)
		self.hlayout6.addWidget(self.sbRepetitions)
		self.pbRepetitions = QPushButton("Repetitions",self)
		self.connect(self.pbRepetitions,SIGNAL('clicked()'),self.repetitionsSlot)
		self.hlayout6.addWidget(self.pbRepetitions)
		self.layout.addLayout(self.hlayout6)

		self.hlayout7 = QHBoxLayout()
		self.lBusy = QLabel("Busy",self)
		self.hlayout7.addWidget(self.lBusy)
		self.rbBLeft = QCheckBox("Left")
		self.rbBRight = QCheckBox("Right")
		self.hlayout7.addWidget(self.rbBLeft)
		self.hlayout7.addWidget(self.rbBRight)
		self.layout.addLayout(self.hlayout7)

		self.hlayout8 = QHBoxLayout()
		self.cbox = QComboBox()
		self.cbox.addItem("hombroDS")
		self.cbox.addItem("hombroDB")
		self.cbox.addItem("hombroIS")
		self.cbox.addItem("hombroIB")
		self.cbox.addItem("codoDS")
		self.cbox.addItem("codoDB")
		self.cbox.addItem("codoIS")
		self.cbox.addItem("codoIB")
		self.cbox.addItem("openNiD")
		self.cbox.addItem("openNiI")
		self.hlayout8.addWidget(self.cbox)
		self.setButton = QPushButton("Set",self)
		self.hlayout8.addWidget(self.setButton)
		self.connect(self.setButton,SIGNAL('clicked()'),self.setMovement)
		self.layout.addLayout(self.hlayout8)

		self.setMinimumSize(300, 450)
		self.job()

	def job(self):
		try:
			self.motors = self.proxy.getAllMotorState()
		except Ice.Exception:
			treceback.print_exc()
		#Upload form
		for key in self.motors.keys():
			self.ledMotors[key].display(self.motors[key].pos)
			
		try:
			if(self.proxy.isBusy("left")):
				self.rbBLeft.setChecked(True)
			else:
				self.rbBLeft.setChecked(False)
		except Ice.Exception:
			treceback.print_exc()

		try:
			if(self.proxy.isBusy("right")):
				self.rbBRight.setChecked(True)
			else:
				self.rbBRight.setChecked(False)
		except Ice.Exception:
			treceback.print_exc()


		##period
		#if(self.time.elapsed() > self.periodSB.value()):
			#self.time.restart()
			#if(self.grabButton.isChecked()):
				#for name in self.states.keys():
					#self.values[self.nameId[name]] = self.states.get(name).pos
				#for value in self.values:
					#self.out << "%.2f" % value <<" "
				#self.out << "\n"
				
			#if(self.grabTButton.isChecked()):
				#for name in self.states.keys():
					#self.values[self.nameId[name]] = self.states.get(name).temperature
				#for value in self.values:
					#self.out << "%.2f" % value <<" "
				#self.out << str(QTime.currentTime().toString("hh:mm:ss:zzz"))<< "\n"
	def arm(self):
		if(self.rbLeft.isChecked()):
			return "left"
		else:
			return "right"

	def speedSlot(self):
		pass
	
	def repetitionsSlot(self):
		try:
			self.proxy.repeat((float)(self.sbSpeed.value()),self.sbRepetitions.value(),self.arm())
		except Ice.Exception:
			treceback.print_exc()

	def reloadSlot(self):
		try:
			self.proxy.reloadMovement(self.arm())
		except Ice.Exception:
			treceback.print_exc()
			
	def loadSlot(self):
		try:
			self.proxy.loadMovement(str(self.lfile.text()),self.arm())
		except Ice.Exception:
			treceback.print_exc()

	def stopSlot(self):
		try:
			self.proxy.stopMovement(self.arm())
		except Ice.Exception:
			treceback.print_exc()

	def goRestSlot(self):
		try:
			self.proxy.goRestPosition(self.arm())
		except Ice.Exception:
			treceback.print_exc()

	def pauseSlot(self):
		if(self.pauseB.isChecked()):
			self.pauseB.setText("Resume")
			if(self.rbLeft.isChecked()):
				self.leftPaused = True
			else:
				self.rightPaused = True
			try:
				self.proxy.pauseMovement(self.arm())
			except Ice.Exception:
				treceback.print_exc()
		else:
			self.pauseB.setText("Pause")
			if(self.rbLeft.isChecked()):
				self.leftPaused = False
			else:
				rightPaused = False
			try:
				self.proxy.resumeMovement(self.arm())
			except Ice.Exception:
				treceback.print_exc()

	def changeArmSlot(self):
		print self.leftPaused,self.rightPaused
		self.pauseB.setText("Pause")
		self.pauseB.setChecked(False)
		if((self.rbLeft.isChecked() and self.leftPaused) or (self.rbRight.isChecked() and self.rightPaused)):
			self.pauseB.setText("Resume")
			self.pauseB.setChecked(True)

	def setMovement(self):
		move = self.mods['RoboCompUrsusArm'].Movement.hombroDS
		if(self.cbox.currentText() == "hombroDS"):
			move = self.mods['RoboCompUrsusArm'].Movement.hombroDS
		elif(self.cbox.currentText() == "hombroDB"):
			move = self.mods['RoboCompUrsusArm'].Movement.hombroDB
		elif(self.cbox.currentText() == "hombroIS"):
			move = self.mods['RoboCompUrsusArm'].Movement.hombroIS			
		elif(self.cbox.currentText() == "hombroIB"):
			move = self.mods['RoboCompUrsusArm'].Movement.hombroIB
		elif(self.cbox.currentText() == "codoIS"):
			move = self.mods['RoboCompUrsusArm'].Movement.codoIS
		elif(self.cbox.currentText() == "codoIB"):
			move = self.mods['RoboCompUrsusArm'].Movement.codoIB
		elif(self.cbox.currentText() == "codoDS"):
			move = self.mods['RoboCompUrsusArm'].Movement.codoDS
		elif(self.cbox.currentText() == "codoDB"):
			move = self.mods['RoboCompUrsusArm'].Movement.codoDB
		elif(self.cbox.currentText() == "openNiD"):
			move = self.mods['RoboCompUrsusArm'].Movement.openNiD
		elif(self.cbox.currentText() == "openNiI"):
			move = self.mods['RoboCompUrsusArm'].Movement.openNiI
		try:
		    self.proxy.setMovement(move)
		except Ice.Exception:
		    treceback.print_exc()
		  
