#
# Copyright (C) 2016 by YOUR NAME HERE
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

import sys, os, Ice

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"

Ice.loadSlice(preStr+"RCMaster.ice")
from RoboCompRCMaster import *


class rcmasterI(rcmaster):
	def __init__(self, worker):
		self.worker = worker

	def updateDb(self, components, c):
		return self.worker.updateDb(components)
	def registerComp(self, compInfo, monitor, assignPort, c):
		return self.worker.registerComp(compInfo, monitor, assignPort)
	def getComps(self, filter, c):
		return self.worker.getComps(filter)
	def getComPort(self, compName, privateIP, c):
		return self.worker.getComPort(compName, privateIP)
	def getComp(self, compName, privateIP, c):
		return self.worker.getComp(compName, privateIP)
	def flush(self, maindb, c):
		return self.worker.flush(maindb)
	def getStat(self, showNames, c):
		return self.worker.getStat(showNames)
