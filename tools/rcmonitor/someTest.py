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

# Custom Template
import Ice
import sys

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.Qt import *

class C(QWidget):
  def __init__(self, endpoint, modules):
    QWidget.__init__(self)
    self.ic = Ice.initialize(sys.argv)
    self.mods = modules
    self.prx = self.ic.stringToProxy(endpoint)
    self.proxy = self.mods['RoboCompCamara'].CamaraPrx.checkedCast(self.prx)
    self.measures = range(33)
    self.job()

  def job(self):
    # Remote procedure call
    output = self.proxy.getRGBPackedImage(5) # vector, head, bState
    # Store image
    self.image = output[0]
    # Store pos measure
    self.measures.pop(0)
    self.measures.append(output[1].tilt.pos)

  def paintEvent(self, event=None):
    painter = QPainter(self)
    painter.setRenderHint(QPainter.Antialiasing, True)
    # Draw image
    qimage = QImage(self.image, 320, 240, QImage.Format_RGB888)
    painter.drawImage(QPointF(0, 0), qimage)
    # Draw signal
    for idx in range(len(self.measures)-1):
      painter.drawLine(idx*10, (self.height()/2)-(self.measures[idx]*100), (idx+1)*10, (self.height()/2)-(self.measures[idx+1]*100))

    painter.end()
