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
import Ice, sys
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class C(QWidget):
  def __init__(self, endpoint, modules):
    QWidget.__init__(self)
    self.ic = Ice.initialize(sys.argv)
    self.mods = modules
    self.prx = self.ic.stringToProxy(endpoint)
    self.proxy = self.mods['RoboCompSlamLaser'].SlamLaserPrx.checkedCast(self.prx)

    self.frame = QFrame(self)
    self.frame.setFixedSize(320,240)
    self.pbGet = QPushButton("Get map",self)
    self.pbGet.resize(150, 30)
    self.pbGet.show()
    self.ctable=[]
    for i in range(0,255):
	    self.ctable.append(qRgb ( i,i,i ))
    
    
    self.gridMap = self.mods['RoboCompSlamLaser'].GridMap()
    self.pose = self.mods['RoboCompSlamLaser'].Pose2D()
    self.connect(self.pbGet,SIGNAL('clicked()'),self.getMap)
    self.show()
  def job(self):
    pass

  def getMap(self):
    print ("clicked")
    try:
        self.gridMap , self.pose = self.proxy.getWholeGrid()
        print (self.gridMap.params.width)
    except Ice.Exception:
        print ('SlamLaser: Error: Map not available.')
        traceback.print_exc()
    self.frame.setFixedSize(self.gridMap.params.width,self.gridMap.params.height)
  
  def paintEvent(self, event=None):
    painter = QPainter(self)
    qimage = QImage(self.gridMap.data,self.gridMap.params.width,self.gridMap.params.height,QImage.Format_Indexed8)
    qimage.setColorTable( self.ctable)
    painter.setRenderHint(QPainter.Antialiasing, True)
    painter.drawImage(QPointF(self.frame.frameRect().x(),self.frame.frameRect().y()), qimage)
    painter.end()
