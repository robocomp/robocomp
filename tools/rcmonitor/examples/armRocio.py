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

import Ice, sys, math, traceback

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *
from ui_armRocioDlg import Ui_ArmRocioDlg


class C(QWidget):
  def __init__(self, endpoint, modules):
	QWidget.__init__(self)
	print "init"
	self.ui = Ui_ArmRocioDlg()
	self.ui.setupUi(self)
	self.t = 0.
	self.ic = Ice.initialize(sys.argv)
	self.mods = modules
	self.prx = self.ic.stringToProxy(endpoint)
	print endpoint
	self.proxy = self.mods['RoboCompArmrocio'].ArmrocioPrx.checkedCast(self.prx)
	self.show()
	#Init comboBox
	self.ui.cbPose.addItem("reposo")
	self.ui.cbPose.addItem("game1_up")
	self.ui.cbPose.addItem("game1_down")
	self.ui.cbPose.addItem("game2_up")
	self.ui.cbPose.addItem("game2_down")
	self.ui.cbPose.addItem("hola1")
	self.ui.cbPose.addItem("hola2")
	#Init SpinBox
	try:
		map = self.proxy.getJointState()
		params = self.proxy.getJointParams()
		#self.ui.sbS1.setValue(map['s1'].pos)
		#self.ui.sbS2.setValue(map['s2'].pos)
		#self.ui.sbS3.setValue(map['s2'].pos)
		#self.ui.sbS4.setValue(map['s3'].pos)
		self.ui.sbSpeed.setValue(params[0].maxVelocity)
		self.ui.lcdSpeed.display(params[0].maxVelocity)
	except Ice.Exception:
		traceback.print_exc()
	
	self.connect( self.ui.pbSetPose, SIGNAL('clicked()'), self.doSetPose )
	self.connect( self.ui.pbReposo, SIGNAL('clicked()'), self.doReposo )
	self.connect( self.ui.sbSpeed, SIGNAL('valueChanged(double)'),self.doSetSpeed )
	self.job()
	
  def job(self):
	try:
		map = self.proxy.getJointState()
		#self.ui.lcdS1.display(map['s1'].pos)
	#self.ui.lcdS2.display(map['s2'].pos)
		#self.ui.lcdS3.display(map['s2'].pos)
		#self.ui.lcdS4.display(map['s3'].pos)
		isMoving = self.proxy.isMoving()
		self.ui.rbPose.setChecked(isMoving);
		pose = self.proxy.getNextPose()
		self.ui.lePose.setText(self.enumToText(pose),"right")
	except Ice.Exception:
		traceback.print_exc()

  def doSetPose(self):
	pose = self.textToEnum(self.ui.cbPose.currentText())
	print pose
	try:
	  self.proxy.setPose( pose )
	except Ice.Exception:
	  traceback.print_exc()
	  
  def doReposo(self):	
	try:
	  self.proxy.setPose(self.mods['RoboCompArmrocio'].Pose.reposo)
	except Ice.Exception:
	  traceback.print_exc()
	  
  def doSetSpeed(self):
	try:
	  self.proxy.setMaxSpeed(self.ui.sbSpeed.value())
	  self.ui.lcdSpeed.display(self.ui.sbSpeed.value())
	except Ice.Exception:
	  traceback.print_exc()

  def enumToText(self,pose):
		if(pose == self.mods['RoboCompArmrocio'].Pose.reposo):
			return "Reposo"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.game1Up):
			return "Game1_Up"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.game1Down):
			return "Game1_Down"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.game2Up):
			return "Game2_Up"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.game2Down):
			return "Game2_Down"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.unknown):
			return "Unknown"
		elif(pose == self.mods['RoboCompArmrocio'].Pose.hola1):
			return "hola1"			
		elif(pose == self.mods['RoboCompArmrocio'].Pose.hola2):
			return "hola2"
		
  def textToEnum(self,pos):
		if(pos == "reposo"):
			return self.mods['RoboCompArmrocio'].Pose.reposo
		elif(pos == "game1_up"):
			return self.mods['RoboCompArmrocio'].Pose.game1Up
		elif(pos == "game1_down"):
			return self.mods['RoboCompArmrocio'].Pose.game1Down
		elif(pos == "game2_up"):
			return self.mods['RoboCompArmrocio'].Pose.game2Up
		elif(pos == "game2_down"):
			return self.mods['RoboCompArmrocio'].Pose.game2Down
		elif(pos == "hola1"):
			return self.mods['RoboCompArmrocio'].Pose.hola1
		elif(pos == "hola2"):
			return self.mods['RoboCompArmrocio'].Pose.hola2

