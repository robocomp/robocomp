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
from ui_headt2pDlg import Ui_HeadT2PDlg


class C(QWidget):
  def __init__(self, endpoint, modules):
	QWidget.__init__(self)
	print "init"
	self.ui = Ui_HeadT2PDlg()
	self.ui.setupUi(self)
	self.t = 0.
	self.ic = Ice.initialize(sys.argv)
	self.mods = modules
	print "init2"
	self.prx = self.ic.stringToProxy(endpoint)
	print endpoint
	self.proxy = self.mods['RoboCompHeadT2P'].HeadT2PPrx.checkedCast(self.prx)
	print "init3"
	self.show()
	print "init4"
	#Init spinboxes values
	map = self.proxy.getHeadState()

	self.ui.sbLeftPan.setValue(map.motorsState["leftPan"].pos)
	self.ui.sbRightPan.setValue(map.motorsState["rightPan"].pos)
	self.ui.sbTilt.setValue(map.motorsState["tilt"].pos)
	
	self.connect( self.ui.pbSaccadic2DLeft, SIGNAL('clicked()'), self.doSaccadic2DLeft )
	self.connect( self.ui.pbSaccadic2DRight, SIGNAL('clicked()'), self.doSaccadic2DRight )
	self.connect( self.ui.pbSaccadic3D, SIGNAL('clicked()'), self.doSaccadic3D )
	self.connect( self.ui.pbReset, SIGNAL('clicked()'), self.doReset );
	self.job()
	
  def job(self):
	map = self.proxy.getHeadState()
	self.ui.lcdLeftPan.display(map.motorsState["leftPan"].pos);
	self.ui.lcdRightPan.display(map.motorsState["rightPan"].pos);
	self.ui.lcdTilt.display(map.motorsState["tilt"].pos);

  def doSaccadic2DLeft(self):	
	try:
	  self.proxy.saccadic2DLeft( self.ui.sbLeftPan.value(), self.ui.sbTilt.value())
	except Ice.Exception:
	  treceback.print_exc()
	  
  def doSaccadic2DRight(self):	
	try:
	  self.proxy.saccadic2DRight( self.ui.sbRightPan.value(),  self.ui.sbTilt.value())
	except Ice.Exception:
	  treceback.print_exc()
	
  def doSaccadic3D(self):	
	try:
	  self.proxy.saccadic3D( self.ui.sbLeftPan.value(),  self.ui.sbRightPan.value(),  self.ui.sbTilt.value())
	except Ice.Exception:
	  treceback.print_exc()
	  
  def doReset(self):	
	try:
	  self.proxy.resetHead()
	except Ice.Exception:
	  treceback.print_exc()
	  
