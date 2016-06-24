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

import sys, os, Ice, traceback, time, socket

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
		self.compdb = RoboCompRCDNS.compDB()

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
		return True

	def checkComp(self, comp):
		#check valid name
		if comp.name == "":
			return False
		#check valid host
		if comp.host.hostName == "":
			return False
		#check valid interfaces
		pass

	def get_open_port():
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
        	s.bind(("",0))
		except socket.error , msg:
    		print 'Cant assign port. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    		return -1
        port = s.getsockname()[1]
        s.close()
        return port

################################
############ Servents ##########
################################

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
		'''
		register a compoenntand assagin a port to it
		'''
		idata = interfaceData()
		if not checkComp(compInfo):
			print "Not a valid component"

		for comp in self.compdb:
			if comp.name == compInfo.name:
				print comp.name,'already exit in host',comp.host.hostName,'with interfaces',comp.interfaces
				return idata
		
		for interfaceName in compInfo.interfaces:
			port = get_open_port()
			if port != -1:
				compInfo.interfaces[interfaceName] = port
			else:
				print "ERROR:Cant assign port to all interfaces"
				return idata
			
		self.compdb.append(compInfo)
		print 'New component registred: ',comp.host,comp.idComp,comp.port
		idata = compInfo.interfaces
		return idata


	#
	# getComps
	#
	def getComps(self, filter, block):
		#
		# YOUR CODE HERE
		#
		comps = compDB()
		return comps


	#
	# getComPort
	#
	def getComPort(self, compName, hostName, block):
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





