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

import Ice, sys, math, traceback, random, copy,time

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompJointMotor'].JointMotorPrx.checkedCast(self.prx)
		self.show()
		self.maxVel = math.pi / 8.
		self.rng = math.pi / 8.

		#TEST ZONE
		self.testWidget = QGroupBox(self)
		self.testWidget.resize(250, 240)
		self.testWidget.move(0, 0)
		self.testWidget.setTitle('Test pane')
		self.testWidget.show()
		
		#Check box of velocity/position test
		self.positionMode = True
		self.checkPositionMode = QCheckBox(self.testWidget)
		self.checkPositionMode.show()
		self.checkPositionMode.setText("Position Mode")
		self.checkPositionMode.move(90, 10)
		self.checkPositionMode.resize(150, 40)
		self.checkPositionMode.setCheckState(0)
		self.checkPositionMode.setChecked(True)
		self.connect(self.checkPositionMode, SIGNAL('stateChanged(int)'), self.switchMode)
		
		self.spinBoxTestVLabel = QLabel(self.testWidget)
		self.spinBoxTestVLabel.setText("Max.\nVelocity")
		self.spinBoxTestVLabel.resize(90, 70)
		self.spinBoxTestVLabel.move(0, 30)
		self.spinBoxTestVLabel.show()
		
		self.spinBoxTestV = QDoubleSpinBox(self.testWidget)
		self.spinBoxTestV.show()
		self.spinBoxTestV.move(90,50)
		self.spinBoxTestV.resize(160, 40)
		self.spinBoxTestV.setMaximum(math.pi*2.)
		self.spinBoxTestV.setMinimum(-math.pi*2.)
		self.spinBoxTestV.setSingleStep(0.1)
		self.spinBoxTestV.setValue(self.maxVel)
		
		
		self.spinBoxTestP = QSpinBox(self.testWidget)
		self.spinBoxTestP.show()
		self.spinBoxTestP.move(90,95)
		self.spinBoxTestP.resize(160, 40)
		self.spinBoxTestP.setMaximum(10000)
		self.spinBoxTestP.setMinimum(10.)
		self.spinBoxTestP.setSingleStep(5)
		self.spinBoxTestP.setValue(5000)
		
		self.spinBoxTestPLabel = QLabel(self.testWidget)
		self.spinBoxTestPLabel.setText("Period")
		self.spinBoxTestPLabel.resize(90, 50)
		self.spinBoxTestPLabel.move(0, 95)
		self.spinBoxTestPLabel.show()
		self.spinBoxTestR = QDoubleSpinBox(self.testWidget)
		self.spinBoxTestR.show()
		self.spinBoxTestR.move(90,140)
		self.spinBoxTestR.resize(160, 40)
		self.spinBoxTestR.setMaximum(2.*math.pi)
		self.spinBoxTestR.setMinimum(-2.*math.pi)
		self.spinBoxTestR.setSingleStep(0.02)
		self.spinBoxTestR.setValue(self.rng)
		self.spinBoxTestRLabel = QLabel(self.testWidget)
		self.spinBoxTestRLabel.setText("Range (+/-)")
		self.spinBoxTestRLabel.resize(90, 50)
		self.spinBoxTestRLabel.move(0, 140)
		self.spinBoxTestRLabel.show()

		self.testActive = False
		self.checkActive = QCheckBox(self.testWidget)
		self.checkActive.show()
		self.checkActive.setText("Activate test")
		self.checkActive.move(50, 175)
		self.checkActive.resize(150, 40)
		self.checkActive.setCheckState(0)
		self.connect(self.checkActive, SIGNAL('stateChanged(int)'), self.switchTest)
		
		#Check box of symetric test
		self.testSymetric = False
		self.checkSymetric = QCheckBox(self.testWidget)
		self.checkSymetric.show()
		self.checkSymetric.setText("Symetric move")
		self.checkSymetric.move(50, 205)
		self.checkSymetric.resize(150, 40)
		self.checkSymetric.setCheckState(0)
		self.connect(self.checkSymetric, SIGNAL('stateChanged(int)'), self.switchSymetric)
		self.timer = QTimer()
		self.timer.start(self.spinBoxTestP.value())
		self.connect(self.timer, SIGNAL('timeout()'), self.test)
		
		


		self.connect(self.spinBoxTestV, SIGNAL('valueChanged(double)'), self.velChan)
		self.connect(self.spinBoxTestP, SIGNAL('valueChanged(int)'), self.perChan)
		self.connect(self.spinBoxTestR, SIGNAL('valueChanged(double)'), self.rngChan)
		
		#COMMAND ZONE
		self.commandWidget = QGroupBox(self)
		self.commandWidget.resize(250, 250)
		self.commandWidget.move(0, 240)
		self.commandWidget.setTitle('Single command pane')
		self.commandWidget.show()
		self.combo = QComboBox(self.commandWidget)
		self.combo.show()
		self.combo.move(0, 30)
		self.combo.resize(250, 40)
		self.readLabel = QLabel(self.commandWidget)
		self.readLabel.show()
		self.readLabel.move(0,self.combo.y()+self.combo.height())
		self.readLabel.resize(250, 40)
		self.spinBoxCommand = QDoubleSpinBox(self.commandWidget)
		self.spinBoxCommand.show()
		self.spinBoxCommand.move(0,self.readLabel.y()+self.readLabel.height())
		self.spinBoxCommand.resize(250, 40)
		self.spinBoxCommand.setMaximum(1024.)
		self.spinBoxCommand.setMinimum(-200.)
		self.spinBoxCommand.setSingleStep(0.1)

		self.commandButton = QPushButton("Set", self.commandWidget)
		self.commandButton.show()
		self.commandButton.move(0,self.spinBoxCommand.y()+self.spinBoxCommand.height())
		self.connect(self.commandButton, SIGNAL('clicked()'), self.clicked)
		self.commandButton.resize(250, 40)

		self.velocityButton = QPushButton("Velocity", self.commandWidget)
		self.velocityButton.show()
		self.velocityButton.move(0,self.commandButton.y()+self.commandButton.height())
		self.connect(self.velocityButton, SIGNAL('clicked()'), self.velocity)
		self.velocityButton.resize(250, 40)



		self.motors = self.proxy.getAllMotorParams()
		self.motorNames = list()
		print ('Motors: ',)
		for item in self.motors:
			print (item.name)
			self.motorNames.append(item.name)
			self.combo.addItem(item.name)
		if len(self.motors)==0: print ('JointMotor: Error: No motors.')

		self.setMinimumSize(250, 440)
		self.states = self.proxy.getAllMotorState()
		
		self.times = 0
		self.synch = 0
		self.refused = 0
		self.job()

	def velChan(self, vel):
		self.maxVel = vel
	def rngChan(self, vel):
		self.rng = vel
	def perChan(self, period):
		self.timer.stop()
		self.timer.start(period)
	def switchTest(self, value):
		self.testActive = not self.testActive
		
	def switchSymetric(self, value):
		self.testSymetric = not self.testSymetric
		
	def switchMode(self, value):
		self.positionMode = not self.positionMode
		print ("Mode switched to"+str(self.positionMode))
	
	def test(self):
		if not self.testActive:
			return
		#~ mod = random.randint(1, 2)
		mod = 2
		if mod == 1: # single saccadic
			if self.positionMode:
				goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
				goal.position = random.uniform(-self.rng, self.rng)
				goal.maxSpeed = self.maxVel
				goal.name = self.motorNames[random.randint(0, len(self.motorNames)-1)]
				try:
					self.proxy.setPosition(goal)
					self.times += 1
					if self.times%10 == 0:
						print ("Sacadic #"+str(self.times), "("+str(self.synch)+") synch")
				except Ice.ConnectionRefusedException:
					self.times = 0
			else:
				goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
				goal.velocity = random.uniform(0, self.maxVel)
				goal.maxAcc = self.maxVel
				goal.name = self.motorNames[random.randint(0, len(self.motorNames)-1)]
				self.proxy.setVelocity( goal )
				
		elif mod == 2: # synchroniced saccadic
			if self.positionMode:
				if self.testSymetric:
					listGoals = list()
					maxMotors = len(self.motorNames)
					motorPool = copy.deepcopy(self.motorNames)
					positions = random.uniform(-self.rng, self.rng)
					for i in range(maxMotors):
						goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
						goal.position = positions
						goal.maxSpeed = self.maxVel
						goal.name = motorPool.pop(random.randint(0, len(motorPool)-1))
						listGoals.append(goal)
					try:
						self.proxy.setSyncPosition(listGoals)
						self.times += 1
						self.synch += 1
						if self.times%10 == 0:
							print ("Sacadic #"+str(self.times), "("+str(self.synch)+") synch")
					except Ice.ConnectionRefusedException:
						self.times = 0
				else:
					listGoals = list()#self.mods['RoboCompJointMotor'].MotorGoalPositionList()
					maxMotors = random.randint(1, len(self.motorNames))
					motorPool = copy.deepcopy(self.motorNames)
					for i in range(maxMotors):
						goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
						goal.position = random.uniform(-self.rng, self.rng)
						goal.maxSpeed = self.maxVel
						goal.name = motorPool.pop(random.randint(0, len(motorPool)-1))
						listGoals.append(goal)
					try:
						self.proxy.setSyncPosition(listGoals)
						self.times += 1
						self.synch += 1
						if self.times%10 == 0:
							print ("Sacadic #"+str(self.times), "("+str(self.synch)+") synch")
					except Ice.ConnectionRefusedException:
						self.times = 0
			else:
				if self.testSymetric:
					listGoals = list()
					maxMotors = len(self.motorNames)
					motorPool = copy.deepcopy(self.motorNames)
					velocity = random.uniform(0, self.maxVel)
					for i in range(maxMotors):
						goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
						goal.velocity = velocity
						goal.maxAcc = self.maxVel
						goal.name = motorPool.pop(random.randint(0, len(motorPool)-1))
						listGoals.append(goal)
					try:
						self.proxy.setSyncVelocity(listGoals)
						self.times += 1
						self.synch += 1
						if self.times%10 == 0:
							print ("Sacadic #"+str(self.times), "("+str(self.synch)+") synch")
					except Ice.ConnectionRefusedException:
						self.times = 0
				else:
					listGoals = list()#self.mods['RoboCompJointMotor'].MotorGoalPositionList()
					maxMotors = random.randint(1, len(self.motorNames))
					motorPool = copy.deepcopy(self.motorNames)
					for i in range(maxMotors):
						goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
						goal.velocity = random.uniform(0, self.maxVel)
						goal.maxAcc = self.maxVel
						goal.name = motorPool.pop(random.randint(0, len(motorPool)-1))
						listGoals.append(goal)
					try:
						self.proxy.setSyncVelocity(listGoals)
						self.times += 1
						self.synch += 1
						if self.times%10 == 0:
							print ("Sacadic #"+str(self.times), "("+str(self.synch)+") synch")
					except Ice.ConnectionRefusedException:
						self.times = 0
		else: # do nothing
			pass

	def job(self):
		#while(True):
			#self.proxy.setPosition(goal1)
			#time.sleep(6)
			#self.proxy.setPosition(goal2)
			#time.sleep(6)
		try:
			self.states = self.proxy.getAllMotorState()
			self.refused = 0
			state = self.states[str(self.combo.currentText())]
			self.readLabel.setText(QString.number(state.pos))
		except Ice.ConnectionRefusedException:
			if self.refused == 0:
				self.refused = 1
				print ('Connection refused')
			elif self.refused == 1:
				self.refused == 2
		#self.clicked()
	
	def clicked(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalPosition()
		goal.position = self.spinBoxCommand.value()
		goal.name = str(self.combo.currentText())
		goal.maxSpeed = self.spinBoxTestV.value()
		print (goal.position, goal.name, goal.maxSpeed)
		self.proxy.setPosition(goal)

	def velocity(self):
		goal = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
		goal.velocity = self.spinBoxTestV.value()
		goal.name = str(self.combo.currentText())
		goalList = list()
		goalList.append(goal)
		goal2 = self.mods['RoboCompJointMotor'].MotorGoalVelocity()
		goal2.name = "r5"
		goal2.velocity = self.spinBoxTestV.value()
		goalList.append(goal2)
			
		print (goalList)
		try:
#			self.proxy.setSyncVelocity(goalList)
			self.proxy.setVelocity(goal)
		except Ice.ConnectionRefusedException:
			self.times = 0
			
