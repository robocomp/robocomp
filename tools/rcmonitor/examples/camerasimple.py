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

# Camera RGB Image Template
import Ice
import sys

class C():
  def __init__(self, endpoint, modules):
    arg = sys.argv
    arg.append("--Ice.MessageSizeMax=4000000")
    self.ic = Ice.initialize(arg)
    self.mods = modules
    self.prx = self.ic.stringToProxy(endpoint)
    self.proxy = self.mods['RoboCompCameraSimple'].CameraSimplePrx.checkedCast(self.prx)
    
  def job(self):
    data = self.proxy.getImage()
    if len(data.image) == 0:
      print ('Error retrieving images!')
    return ['rgbImage', [ data.image, data.width, data.height ] ]


#  def paintEvent(self, event=None):
    
#    painter = QPainter(self)
#    painter.setRenderHint(QPainter.Antialiasing, True)
#    image = QImage(self.color, color_image_width, color_image_height, QImage.Format_RGB888)
#    painter.drawImage(QPointF(self.ui.frameRGB.x(), self.ui.frameRGB.y()), image)

#    painter.end()
