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
from ui_kinectDlg import Ui_KinectDlg


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		print "init"
		self.ui = Ui_KinectDlg()
		self.ui.setupUi(self)
		self.t = 0.
		arg = sys.argv
		arg.append("--Ice.MessageSizeMax=10240")
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		print endpoint
		self.proxy = self.mods['RoboCompKinect'].KinectPrx.checkedCast(self.prx)
		self.show()
		self.ui.cbLedOpt.addItem("OFF")
		self.ui.cbLedOpt.addItem("GREEN")
		self.ui.cbLedOpt.addItem("RED")
		self.ui.cbLedOpt.addItem("YELLOW")
		self.ui.cbLedOpt.addItem("blinkGREEN")
		self.ui.cbLedOpt.addItem("blinkREDYELLOW")

		self.connect( self.ui.pbSetLed, SIGNAL('clicked()'), self.doSetLed )
		self.connect( self.ui.sbTilt, SIGNAL('valueChanged(double)'),self.doSetTilt )

		self.vector = []
		self.job()

	def job(self):
		try:
			self.vector = self.proxy.getDataRGBZinIR() # imageVector, depthVector, headState, baseState
			if len(self.vector) == 0:
				print 'Error retrieving images!'
		except Ice.Exception:
			traceback.print_exc()


	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)

		image = QImage(self.vector[0], 640, 480, QImage.Format_RGB888)
		painter.drawImage(QPointF(self.ui.frame.x(), self.ui.frame.y()), image)

		painter.end()
		#painter = None

	def doSetLed(self):
		ledLight = self.textToEnum(self.ui.cbLedOpt.currentText())
		print ledLight
		try:
			self.proxy.setLed( ledLight )
		except Ice.Exception:
			traceback.print_exc()
			
	def doSetTilt(self):
	   try:
		 self.proxy.setTilt(self.ui.sbTilt.value())
		 self.ui.lcdTilt.display(self.ui.sbTilt.value())
	   except Ice.Exception:
		 traceback.print_exc()
		 
	def textToEnum(self,pos):
		if(pos == "OFF"):
				return self.mods['RoboCompKinect'].LedOptions.OFF
		elif(pos == "GREEN"):
				return self.mods['RoboCompKinect'].LedOptions.GREEN
		elif(pos == "RED"):
				return self.mods['RoboCompKinect'].LedOptions.RED
		elif(pos == "YELLOW"):
				return self.mods['RoboCompKinect'].LedOptions.YELLOW
		elif(pos == "blinkYELLOW"):
				return self.mods['RoboCompKinect'].LedOptions.blinkYELLOW
		elif(pos == "blinkGREEN"):
				return self.mods['RoboCompKinect'].LedOptions.blinkGREEN
		elif(pos == "blinkREDYELLOW"):
				return self.mods['RoboCompKinect'].LedOptions.blinkREDYELLOW

