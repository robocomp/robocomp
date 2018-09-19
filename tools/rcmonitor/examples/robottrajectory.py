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

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.t = 0.
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompRobotTrajectory'].RobotTrajectoryPrx.checkedCast(self.prx)
		self.show()
		
		#Init spinboxes values
		self.pbAdvance = QPushButton("Advance")
		self.sbAdvance = QSpinBox()
		self.sbAdvance.setRange(-2000,2000)
		
		self.pbBack = QPushButton("Back")
		
		self.pbRotate = QPushButton("Rotate")
		self.sbRotate = QDoubleSpinBox()
		self.sbRotate.setRange(-3.0,3.0)
		self.cbRotate = QCheckBox("World turn")
		
		self.layout = QVBoxLayout()
		self.layout.addWidget(self.sbAdvance)
		self.layout.addWidget(self.pbAdvance)
		self.layout.addWidget(self.cbRotate)
		self.layout.addWidget(self.sbRotate)
		self.layout.addWidget(self.pbRotate)
		self.layout.addWidget(self.pbBack)
		self.w = QWidget()
		self.w.show()
		self.layout.addWidget(self.w)


		self.layout2 = QHBoxLayout()
		self.setLayout(self.layout)
		self.w.setLayout(self.layout2)

		self.xLabel = QLabel("x")
		self.xLabel.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
		self.layout2.addWidget(self.xLabel)
		self.xPosition = QDoubleSpinBox()
		self.xPosition.setMinimum(-100000)
		self.xPosition.setMaximum(100000)
		self.layout2.addWidget(self.xPosition)
		self.zLabel = QLabel("z")
		self.zLabel.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
		self.layout2.addWidget(self.zLabel)
		self.zPosition = QDoubleSpinBox()
		self.zPosition.setMinimum(-100000)
		self.zPosition.setMaximum(10000)
		self.layout2.addWidget(self.zPosition)
		self.pbGoto = QPushButton("go to")
		self.layout2.addWidget(self.pbGoto)

		self.connect( self.pbAdvance, SIGNAL('clicked()'), self.Advance );
		self.connect( self.pbBack, SIGNAL('clicked()'), self.Back );
		self.connect( self.pbRotate , SIGNAL('clicked()'), self.Rotate );
		self.connect( self.pbGoto, SIGNAL('clicked()'), self.Goto );
		self.job()
		
		path = list()

		goal = self.mods['RoboCompRobotTrajectory'].Stage()
		goal.x = 4000
		goal.z = 4000
		goal.pureRotation = False
		path.append(goal)
		goal = self.mods['RoboCompRobotTrajectory'].Stage()
		goal.x = 4000
		goal.z = -4000
		goal.pureRotation = False
		path.append(goal)
		goal = self.mods['RoboCompRobotTrajectory'].Stage()
		goal.x = -4000
		goal.z = -4000
		goal.pureRotation = False
		path.append(goal)
		goal = self.mods['RoboCompRobotTrajectory'].Stage()
		goal.x = -4000
		goal.z = 4000
		goal.pureRotation = False
		path.append(goal)
		goal = self.mods['RoboCompRobotTrajectory'].Stage()
		goal.x = 0
		goal.z = 0
		goal.pureRotation = False
		path.append(goal)
		
		self.proxy.setPath(path);

	
	def job(self):
		pass
 
	def Advance(self):
		print self.sbAdvance.value()
		self.proxy.advanceVel(self.sbAdvance.value()*1.0,300.0)
	def Back(self):
		print self.sbAdvance.value()
		self.proxy.backVel(self.sbBack.value()*1.0,150.0)
	def Rotate(self):
		if(self.cbRotate.isChecked()):
			self.proxy.rotateWorldVel(self.sbRotate.value(),1.5)
		else:
			self.proxy.rotateVel(self.sbRotate.value(),1.5)
	def Goto(self):
		print self.xPosition.value(), self.zPosition.value()
		self.proxy.gotoPointVel(self.xPosition.value(), self.zPosition.value(),300.0,1.5)
