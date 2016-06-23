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

import sys, os, Ice, traceback, time

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"RCMaster.ice")
from RoboCompRCMaster import *


from rcmasterI import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True


	#
	# syncwithhost
	#
	def syncwithhost(self, remoteHost):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# updateDb
	#
	def updateDb(self, components):
		#
		# YOUR CODE HERE
		#
		pass


	#
	# registerComp
	#
	def registerComp(self, compInfo, monitor):
		#
		# YOUR CODE HERE
		#
		idata = interfaceData()
		return idata


	#
	# getComps
	#
	def getComps(self, filter):
		#
		# YOUR CODE HERE
		#
		comps = compDB()
		return comps


	#
	# getComPort
	#
	def getComPort(self, compName, hostName):
		ret = int()
		#
		# YOUR CODE HERE
		#
		return ret


	#
	# sync
	#
	def sync(self, sourceHost):
		#
		# YOUR CODE HERE
		#
		db = compDB()
		return db


	#
	# flush
	#
	def flush(self):
		#
		# YOUR CODE HERE
		#
		pass





