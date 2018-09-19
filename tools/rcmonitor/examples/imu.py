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
		self.Lista=list();
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompIMU'].IMUPrx.checkedCast(self.prx)

		self.accLabelx = QLabel("Acc X", self)
		self.accLabelx.show()
		self.accLabelx.move(5,8)
		self.green=QColor("green")
		self.accx = QLCDNumber(self)
		self.accx.show()
		self.accx.setSegmentStyle(QLCDNumber.Flat)
		self.accx.move(45,8)

		self.accLabely = QLabel("Acc Y", self)
		self.accLabely.show()
		self.accLabely.move(5,30)
		self.accy = QLCDNumber(self)
		self.accy.show()
		self.accy.setSegmentStyle(QLCDNumber.Flat)
		self.accy.move(45,30)

		self.accLabelz = QLabel("Acc Z", self)
		self.accLabelz.show()
		self.accLabelz.move(5,52)
		self.accz = QLCDNumber(self)
		self.accz.show()
		self.accz.setSegmentStyle(QLCDNumber.Flat)
		self.accz.move(45,52)
    
		self.velLabelx = QLabel("gyrX", self)
		self.velLabelx.show()
		self.velLabelx.move(130,8)
		self.green=QColor("green")
		self.gyrx = QLCDNumber(self)
		self.gyrx.show()
		self.gyrx.setSegmentStyle(QLCDNumber.Flat)
		self.gyrx.move(165,8)

		self.velLabely = QLabel("gyrY", self)
		self.velLabely.show()
		self.velLabely.move(130,30)
		self.green=QColor("green")
		self.gyry = QLCDNumber(self)
		self.gyry.show()
		self.gyry.setSegmentStyle(QLCDNumber.Flat)
		self.gyry.move(165,30)
		
		self.velLabelz = QLabel("gyrZ", self)
		self.velLabelz.show()
		self.velLabelz.move(130,52)
		self.green=QColor("green")
		self.gyrz = QLCDNumber(self)
		self.gyrz.show()
		self.gyrz.setSegmentStyle(QLCDNumber.Flat)
		self.gyrz.move(165,52)

		self.magLabelx = QLabel("MagX", self)
		self.magLabelx.show()
		self.magLabelx.move(265,8)
		self.green=QColor("green")
		self.magx = QLCDNumber(self)
		self.magx.show()
		self.magx.setSegmentStyle(QLCDNumber.Flat)
		self.magx.move(305,8)

		self.magLabely = QLabel("MagY", self)
		self.magLabely.show()
		self.magLabely.move(265,30)
		self.green=QColor("green")
		self.magy = QLCDNumber(self)
		self.magy.show()
		self.magy.setSegmentStyle(QLCDNumber.Flat)
		self.magy.move(305,30)
		
		self.magLabelz = QLabel("MagZ", self)
		self.magLabelz.show()
		self.magLabelz.move(265,52)
		self.green=QColor("green")
		self.magz = QLCDNumber(self)
		self.magz.show()
		self.magz.setSegmentStyle(QLCDNumber.Flat)
		self.magz.move(305,52)
		
		self.rollLabel = QLabel("ROLL", self)
		self.rollLabel.show()
		self.rollLabel.move(400,8)
		self.green=QColor("green")
		self.roll = QLCDNumber(self)
		self.roll.show()
		self.roll.setSegmentStyle(QLCDNumber.Flat)
		self.roll.move(435,8)
		
		self.pichLabel = QLabel("PICH", self)
		self.pichLabel.show()
		self.pichLabel.move(400,30)
		self.green=QColor("green")
		self.pich = QLCDNumber(self)
		self.pich.show()
		self.pich.setSegmentStyle(QLCDNumber.Flat)
		self.pich.move(435,30)

		self.yawLabel = QLabel("YAW", self)
		self.yawLabel.show()
		self.yawLabel.move(400,52)
		self.green=QColor("green")
		self.yaw = QLCDNumber(self)
		self.yaw.show()
		self.yaw.setSegmentStyle(QLCDNumber.Flat)
		self.yaw.move(435,52)
		
		self.tempLabel = QLabel("TEMP", self)
		self.tempLabel.show()
		self.tempLabel.move(535,52)
		self.green=QColor("green")
		self.temp = QLCDNumber(self)
		self.temp.show()
		self.temp.setSegmentStyle(QLCDNumber.Flat)
		self.temp.move(670,52)

		self.button = QPushButton("reset", self)
		self.button.move(600,20)
		self.connect(self.button, SIGNAL("clicked()"), self.resetSlot)
		self.button.show()
		


		self.show()

	def resetSlot(self):
		self.proxy.resetImu()

	def job(self):

		self.a = self.proxy.getDataImu( )
		self.accx.display( self.a.acc.XAcc )
		self.accy.display( self.a.acc.YAcc )
		self.accz.display( self.a.acc.ZAcc )
		
		self.gyrx.display( self.a.gyro.XGyr )
		self.gyry.display( self.a.gyro.YGyr )
		self.gyrz.display( self.a.gyro.ZGyr )
		
		self.magx.display( self.a.mag.XMag )
		self.magy.display( self.a.mag.YMag )
		self.magz.display( self.a.mag.ZMag )
		
		self.roll.display( self.a.rot.Roll )
		self.pich.display( self.a.rot.Pitch )
		self.yaw.display( self.a.rot.Yaw )

		self.temp.display( self.a.temperature )

