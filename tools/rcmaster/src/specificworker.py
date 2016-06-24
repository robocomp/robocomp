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
		self.compcache = RoboCompRCDNS.compDB()
		self.savebit = False

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
		if savebit:
			self.savedb()
		#ping all componsnts and cache is necc
		return True

	def checkComp(self, comp):
		#check valid name
		if comp.name == "":
			return False
		#check valid host
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		except socket.error, msg:
			print 'Failed to create socket. Error message : ' + msg[1]
		try:
			remote_ip = socket.gethostbyname(comp.host.hostName)
		except socket.gaierror:
			print 'Hostname could not be resolved'
			return False
		if remote_ip != comp.host.publicIP:
			return False
		
		#check valid interfaces
		pass

	def get_open_port(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
        	s.bind(("",0))
		except socket.error , msg:
    		print 'Cant assign port. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    		return -1
        port = s.getsockname()[1]
        s.close()
        return port

    def savedb(self):
    	pass

    def loaddb(self):
    	pass

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
		@TODO monitor, multiple component for oad balencing
		'''
		idata = interfaceData()
		if not checkComp(compInfo):
			print "Not a valid component"
			raise InvalidComponent

		for comp in self.compdb:
			if comp.name == compInfo.name:
				print comp.name,'already exit in host',comp.host.hostName,'with interfaces',comp.interfaces
				raise DuplicateComponent
		
		cacheFound = False
		for cachedcomp in compcache:
			if cachedcomp.name == compInfo.name and cachedcomp.host == compInfo.host and cachedcomp.interfaces.keys() == compInfo.interfaces.keys():
				compInfo.interfaces = cachedcomp.interfaces
				cacheFound = True
				break

		if cacheFound:
			for interfaceName in compInfo.interfaces:
				port = get_open_port()
				if port != -1:
					compInfo.interfaces[interfaceName] = port
				else:
					print "ERROR: Cant assign port to all interfaces"
					raise PortAssignError
		
		self.compdb.append(compInfo)
		self.savebit = True
		print 'New component registred: ',comp.host,comp.idComp
		idata = compInfo.interfaces
		return idata


	#
	# getComps
	#
	def getComps(self, filter, block):
		# @TODO block
		if filter.name != '':
			return [x for x in compdb if x.name == filter.name]
		
		tempdb = self.compdb
		if tempdb.host.name != '':
			tempdb = [x for x in tempdb if x.host.name == filter.host.name]	
		if tempdb.host.publicIP != '':
			tempdb = [x for x in tempdb if x.host.publicIP == filter.host.publicIP]	
		if tempdb.host.privateIP != '':
			tempdb = [x for x in tempdb if x.host.privateIP == filter.host.privateIP]	
		if len(tempdb.interfaces) != 0:
			tempdb = [x for x in tempdb if filter.tempdb.interfaces.keys() in x.tempdb.interfaces.keys()]	
		return tempdb

	#
	# getComPort
	#
	def getComPort(self, compName, hostName, block):
		for comp in compdb:
			if comp.name == compName and comp.host.name == hostName:
				if len(comp.interfaces) != 1:
					raise InvalidComponent
				return comp.interfaces.values()[0]
		raise ComponentNotFound


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





