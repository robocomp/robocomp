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

from PySide.QtCore import *
from PySide.QtGui import *
from ui_jointmotorDlg import Ui_JointMotorDlg

class C(QWidget):
  def __init__(self, endpoint, modules):
	QWidget.__init__(self)
	self.ui = Ui_JointMotorDlg()
	self.ui.setupUi(self)
	self.t = 0.
	self.ic = Ice.initialize(sys.argv)
	self.mods = modules
	self.prx = self.ic.stringToProxy(endpoint)
	self.proxy = self.mods['RoboCompJointMotor'].JointMotorPrx.checkedCast(self.prx)
	self.show;
	
	#Init spinboxes values
	map = self.proxy.getAllMotorState()
	self.ui.sbNeck.setValue(map['neck'].pos)
	self.ui.sbLeftPan.setValue(map['leftPan'].pos)
	self.ui.sbRightPan.setValue(map['rightPan'].pos)
	self.ui.sbTilt.setValue(map['tilt'].pos)
	
	self.connect( self.ui.sbNeck , SIGNAL('valueChanged(double)'), self.ChangeNeck );
	self.connect( self.ui.sbTilt , SIGNAL('valueChanged(double)'), self.ChangeTilt );
	self.connect( self.ui.sbLeftPan , SIGNAL('valueChanged(double)'), self.ChangeLeftPan );
	self.connect( self.ui.sbRightPan , SIGNAL('valueChanged(double)'), self.ChangeRightPan );
	#self.connect( self.ui.pbSaccadic, SIGNAL('clicked()'), self.doSaccadic );
	
	self.job()
	
  def job(self):
	#map = self.proxy.getMotorStateMap(['Neck', 'LeftPan', 'RightPan', 'Tilt']);
	map = self.proxy.getAllMotorState()
	self.ui.lcdNeck.display(map['neck'].pos)
	self.ui.lcdLeftPan.display(map['leftPan'].pos)
	self.ui.lcdRightPan.display(map['rightPan'].pos)
	self.ui.lcdTilt.display(map['tilt'].pos)
	#mState = self.proxy.getMotorState("Neck")
	#mState = self.proxy.getMotorState("LeftPan")
	#self.ui.lcdLeftPan.display(mState.pos);
	#mState = self.proxy.getMotorState("RightPan")
	#self.ui.lcdRightPan.display(mState.pos);
	#mState = self.proxy.getMotorState("Tilt")
	#self.ui.lcdTilt.display(mState.pos);
	##return ['signal', [ mState.pos, 200 ] ]

 
  def ChangeNeck(self, value):
	try:
	  g = self.mods['RoboCompJointMotor'].MotorGoalPosition();
	  g.name = "neck"
	  g.position = value
	  g.maxSpeed = 1.5
	  self.proxy.setPosition( g );
	except Ice.Exception:
	  traceback.print_exc()
	  
  def ChangeTilt(self, value):
	try:
	  g = self.mods['RoboCompJointMotor'].MotorGoalPosition();
	  g.name = "tilt"
	  g.position = value
	  g.maxSpeed = 1.5
	  self.proxy.setPosition( g );
	except Ice.Exception:
	  traceback.print_exc()
	  
  def ChangeLeftPan(self, value):
	try:
	  g = self.mods['RoboCompJointMotor'].MotorGoalPosition();
	  g.name = "leftPan"
	  g.position = value
	  g.maxSpeed = 1.5
	  self.proxy.setPosition( g )
	except Ice.Exception:
	  traceback.print_exc()
	  
  def ChangeRightPan(self, value):
	try:
	  g = self.mods['RoboCompJointMotor'].MotorGoalPosition();
	  g.name = "rightPan"
	  g.position = value
	  g.maxSpeed = 1.5
	  self.proxy.setPosition(g )
	except  Ice.Exception:
	  traceback.print_exc()
	  
  #def doSaccadic(self):
	#g = self.mods['RoboCompHeadNTP'].MotorGoalPosition();
	#gl = self.mods['RoboCompHeadNTP'].MotorGoalPositionList();
	
	#g.name = "Neck"
	#g.maxSpeed = 0.0
	#g.position = self.ui.sbNeck.value()
	#gl.append( g )
	
	#g.name = "Tilt"
	#g.position = self.ui.sbTilt.value()
	#gl.append( g )
	
	#g.name = "LeftPan"
	#g.position = self.ui.sbLeftPan.value()
	#gl.append( g )
	
	#g.name = "RightPan"
	#g.position = self.ui.sbRightPan.value()
	#gl.append( g )
	
	#try:
	  #self.proxy.setSyncPosition( gl );
	#except Ice.Exception:
	  #treceback.print_exc()

