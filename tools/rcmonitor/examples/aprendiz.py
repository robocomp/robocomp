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
from ui_aprendizDlg import Ui_AprendizDlg


class C(QWidget):
  def __init__(self, endpoint, modules):
	QWidget.__init__(self)
	self.ui = Ui_AprendizDlg()
	self.ui.setupUi(self)
	self.t = 0.
	self.ic = Ice.initialize(sys.argv)
	self.mods = modules
	self.prx = self.ic.stringToProxy(endpoint)
	self.proxy = self.mods['RoboCompAprendiz'].AprendizPrx.checkedCast(self.prx)
	self.show;

	self.connect( self.ui.pbSend, SIGNAL('clicked()'), self.doSend );
	self.connect( self.ui.sbX, SIGNAL('valueChanged(double)'), self.doSend );
	self.connect( self.ui.sbY, SIGNAL('valueChanged(double)'), self.doSend );
	self.connect( self.ui.sbZ, SIGNAL('valueChanged(double)'), self.doSend );
	self.job()
	
  def job(self):
	pass

	  
  def doSend(self):	
	try:
	
		c=self.mods['RoboCompAprendiz'].coor3D()
		c.x=float(self.ui.sbX.value())
		c.y=float(self.ui.sbY.value())
		c.z=float(self.ui.sbZ.value())
		self.proxy.setWrist(c)
	  
	except Ice.Exception:
	  treceback.print_exc()
	  
