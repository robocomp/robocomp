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
from ui_forkDlg import Ui_forkDlg


class C(QWidget):
  def __init__(self, endpoint, modules):
	QWidget.__init__(self)
	print "init"
	self.ui = Ui_forkDlg()
	self.ui.setupUi(self)
	self.t = 0.
	self.ic = Ice.initialize(sys.argv)
	self.mods = modules
	print "init2"
	self.prx = self.ic.stringToProxy(endpoint)
	print endpoint
	self.proxy = self.mods['RoboCompFork'].ForkPrx.checkedCast(self.prx)
	print "init3"
	self.show()
	print "init4"
	#Init spinboxes values
	
	self.connect( self.ui.pbSetZeroPos, SIGNAL('clicked()'), self.setZeroPos )
	self.connect( self.ui.pbGoUp, SIGNAL('clicked()'), self.goUp )
	self.connect( self.ui.pbGoDown, SIGNAL('clicked()'), self.goDown )
	self.connect( self.ui.pbStop, SIGNAL('clicked()'), self.stop );
	self.connect( self.ui.pbSpeed, SIGNAL('clicked()'), self.setSpeed );
	self.connect( self.ui.pbGoPos, SIGNAL('clicked()'), self.setPos );
	self.job()
	
  def job(self):
	pos = self.proxy.getPosition()
	self.ui.lcdPos.display(pos);
	
  def setZeroPos(self):	
	try:
	  self.proxy.setUpPosition()
	except Ice.Exception:
	  treceback.print_exc()
	  
  def goUp(self):	
	try:
	  self.proxy.goUp( )
	except Ice.Exception:
	  treceback.print_exc()
	
  def goDown(self):	
	try:
	  self.proxy.goDown()
	except Ice.Exception:
	  treceback.print_exc()
	  
  def stop(self):	
	try:
	  self.proxy.stop()
	except Ice.Exception:
	  treceback.print_exc()
	  
  def setSpeed(self):	
	try:
	  self.proxy.setSpeed(self.ui.sbSpeed.value())
	except Ice.Exception:
	  treceback.print_exc()

  def setPos(self):	
	try:
	  self.proxy.setPosition(self.ui.sbPos.value())
	except Ice.Exception:
	  treceback.print_exc()


