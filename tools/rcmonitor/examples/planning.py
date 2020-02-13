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
import random

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


domain=open('domain.pddl', 'r').read()
problem=open('problem.pddl', 'r').read()

class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ic = Ice.initialize(sys.argv)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompPlanning'].PlanningPrx.checkedCast(self.prx)
		self.show()

		self.commandButton = QPushButton("Test", self)
		self.commandButton.resize(200, 50)
		self.commandButton.show()
		self.commandButton.move(5,5)
		self.connect(self.commandButton, SIGNAL('clicked()'), self.clicked )

		self.hlayout = QHBoxLayout(self)
		self.hlayout.addWidget(self.commandButton)
		
		
		self.setMinimumSize(210, 80)
		self.setMaximumSize(210, 80)

	def job(self):
		pass
		#self.clicked()

	def clicked(self):
		ret = False
		try:
			print ("<<<")
			ret, plan = self.proxy.getSolution(domain, problem)
			print (">>>")
		except Ice.Exception:
			traceback.print_exc()
			return

		if ret:
			print ("Plan found!")
			print ("Cost:", plan.cost)
			for action in plan.actions:
				print (action.name, ": ",)
				for sym in action.symbols:
					print (sym,)
				print ('')
		else:
			print ("No plan found")

