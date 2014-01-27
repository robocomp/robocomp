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


import Ice
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *

class C(QWidget):
  def __init__(self, endpoint, modules):
    QWidget.__init__(self)
    self.ic = Ice.initialize(sys.argv)
    self.mods = modules
    self.prx = self.ic.stringToProxy(endpoint)
    self.proxy = self.mods['RoboCompCubafeatures'].CubafeaturesPrx.checkedCast(self.prx)
    self.job()

  def drawCircle(self, painter, x, y, diam):
    painter.drawEllipse(0.1*x+self.width()/2 - 0.5*diam, -0.1*y+self.height()/2 -0.5*diam, diam, diam)

  def drawLineCUBA(self, painter, inix, iniy, finx, finy, diam):
    painter.drawLine(0.1*inix+self.width()/2 - 0.5*diam, -0.1*iniy+self.height()/2 -0.5*diam, 
      0.1*finx+self.width()/2 - 0.5*diam, -0.1*finy+self.height()/2 - 0.5*diam)

  def drawCenter(self, painter):
    wid = 0
    while wid < 20:
      self.drawCircle(painter, self.width()/2, self.height()/2, wid)
      wid += 2

  def job(self):
    self.features = self.proxy.getFeatures()

  def paintEvent(self, event=None):
    painter = QPainter(self)
    painter.setRenderHint(QPainter.Antialiasing, True)

    painter.setPen(Qt.red)
    self.drawCenter(painter)
    
    painter.setPen(Qt.blue)
    print len(self.features.p)
    for point in self.features.p:
      print point.x, point.z
      self.drawCircle(painter, point.x, point.z, 14)

    painter.setPen(Qt.green)
    for segment in self.features.s:
      print segment.p1x, segment.p1z, segment.p2x, segment.p2z
      self.drawLineCUBA(painter, segment.p1x, segment.p1z, segment.p2x, segment.p2z, 1)

    painter.end()

