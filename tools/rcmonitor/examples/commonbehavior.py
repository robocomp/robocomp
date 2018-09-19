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
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompCommonBehavior'].CommonBehaviorPrx.checkedCast(self.prx)
		
		self.layout = QHBoxLayout(self)
		self.vlayout = QVBoxLayout()
		self.pbKill = QPushButton("kill", self)
		self.connect(self.pbKill, SIGNAL("clicked()"), self.kill)

		self.vlayout.addWidget(self.pbKill)
		self.sLabel = QLabel("State:")
		self.hlayout = QHBoxLayout()
		self.hlayout.addWidget(self.sLabel)
		self.ssLabel = QLabel("")
		self.hlayout.addWidget(self.ssLabel)
		self.vlayout.addLayout(self.hlayout)
		
		self.periodLabel = QLabel("Period",self)
		self.vlayout.addWidget(self.periodLabel)
		self.sbFreq = QSpinBox(self)
		self.sbFreq.setMaximum(3000)
		self.sbFreq.setValue(self.proxy.getPeriod())
		self.connect(self.sbFreq,SIGNAL('valueChanged(int)'),self.changePeriod)
		self.vlayout.addWidget(self.sbFreq)

		self.pbConfig = QPushButton("Reload Config",self)
		self.connect(self.pbConfig,SIGNAL('clicked()'),self.reloadconfig)
		self.vlayout.addWidget(self.pbConfig)

		self.pbSetParameters = QPushButton("Set parameters",self)
		self.connect(self.pbSetParameters,SIGNAL('clicked()'),self.setparameters)
		self.vlayout.addWidget(self.pbSetParameters)
		
		self.spacer = QSpacerItem(10,10,QSizePolicy.Minimum,QSizePolicy.Expanding)
		self.vlayout.addItem(self.spacer)
		self.layout.addLayout(self.vlayout)

		#formulario parametros
		self.params = self.proxy.getParameterList()
		self.layoutF = QVBoxLayout()
		self.lparams = []
		for i in self.params:
			self.l = QHBoxLayout()
			self.label = QLabel(i,self)
			self.l.addWidget(self.label)
			self.lparams.append( QLineEdit(self.params[i].value,self))
			self.lparams[-1].setReadOnly(not self.params[i].editable)
			self.l.addWidget(self.lparams[-1])
			self.layoutF.addLayout(self.l)

		self.layout.addLayout(self.layoutF)
		self.show()

	def job(self):
		if self.proxy.getState() == self.mods['RoboCompCommonBehavior'].State.Running:
			self.ssLabel.setText("Running")
		else:
			self.ssLabel.setText("Starting")


	def changePeriod(self,a):
		self.proxy.setPeriod(a)

	def kill(self):
		self.proxy.killYourSelf()
		
	def reloadconfig(self):
		self.proxy.reloadConfig()
		self.loadparameters()

	def loadparameters(self):
		self.params = self.proxy.getParameterList()
		pos = 0
		for i in self.params:
			self.lparams[pos].setText(self.params[i].value)
			pos += 1

	def setparameters(self):
		pos = 0
		for i in self.params:
			if self.params[i].editable == True:
				self.params[i].value = str(self.lparams[pos].text())
			pos +=1
		self.proxy.setParameterList(self.params)
		self.loadparameters()
