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

Ice.loadSlice(preStr+"rcmaster.ice")
from RoboCompRCMaster import *

class rcmasterI(rcmaster):
	def __init__(self, worker):
		self.worker = worker

	def syncwithhost(self, remoteHost, c):
		return self.worker.syncwithhost(remoteHost)
	def updateDb(self, components, c):
		return self.worker.updateDb(components)
	def registerComp(self, compInfo, monitor, c):
		return self.worker.registerComp(compInfo, monitor)
	def getComps(self, filter, block, c):
		return self.worker.getComps(filter, block)
	def getComPort(self, compName, hostName, block, c):
		return self.worker.getComPort(compName, hostName, block)
	def sync(self, sourceHost, c):
		return self.worker.sync(sourceHost)
	def flush(self, c):
		return self.worker.flush()





