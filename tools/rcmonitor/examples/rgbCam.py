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
    self.proxy = self.mods['RoboCompCamera'].CameraPrx.checkedCast(self.prx)
    self.params = self.proxy.getCamParams()
    self.cameraNum = 0
    if self.params.numCams > 1:
      self.cameraNum = self.params.bothCameras
  def job(self):
    vector, hState, bState = self.proxy.getRGBPackedImage(self.cameraNum) # imageVector, headState, baseState
    if len(vector) == 0:
     print 'Error retrieving images!'
    return ['rgbImage', [ vector, self.params.width, self.params.height*self.params.numCams ] ]
