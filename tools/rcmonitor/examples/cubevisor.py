# -*- coding: utf-8 -*-
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
    self.proxy = self.mods['RoboCompCubevisor'].CubevisorPrx.checkedCast(self.prx)
    
    self.Label = QLabel("Numero de habitaciones detectadas", self)
    self.Label.show()
    self.Label.move(120,80)
    self.NumberOfRooms = QLCDNumber(self)
    self.NumberOfRooms.show()
    self.NumberOfRooms.setSegmentStyle(QLCDNumber.Flat)
    self.NumberOfRooms.move(350,80)
  def job(self):
    nModels = self.proxy.getNumberOfModels()
    self.NumberOfRooms.display(nModels)
    