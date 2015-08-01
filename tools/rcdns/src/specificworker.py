#
# Copyright (C) 2015 by YOUR NAME HERE
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

import sys, os, Ice, traceback

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
Ice.loadSlice(preStr+"rcdns.ice")
from RoboCompRCDNS import *
import RoboCompRCDNS

from rcdnsI import *

     
        
class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.portInit = 10000
		self.maxPortUse = 10000
		self.currentPort = 0
		self.comps = []
		
	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute... We have ',len(self.comps),' components'
		self.getAllCompsHost('robonuc1.local')
		return True







########## SERVANTS

	#
	# giveMePort
	#
	def giveMePort(self, idComp, host):
		if self.currentPort < self.portInit+self.maxPortUse:
			for comp in self.comps:
				if comp.idComp == idComp:
					print comp.idComp,'already exit in host',comp.host,'with port',comp.port
					return 0
				
			comp = RoboCompRCDNS.DNSdata()
			comp.host = host
			comp.idComp = idComp
			comp.port = self.portInit + self.currentPort
			self.comps.append(comp)
			self.currentPort += 1
			print 'New component in DNS: ',comp.host,comp.idComp,comp.port
		else:
			print 'LIMIT of use ports'
		return comp.port


	#
	# getAllCompsHost
	#
	def getAllCompsHost(self, host):
		listComp = []
		a = RoboCompRCDNS.DNSdata()
		for comp in self.comps:
			if comp.host.publicIP == host or comp.host.privateIP == host or comp.host.hostName == host:
				listComp.append(comp)
		print 'Returning all comps in:',host
		return listComp


	#
	# getAllComps
	#
	def getAllComps(self):
		print 'Returning all comps in DNS'
		return self.comps


	#
	# getComponentId
	#
	def getComponentId(self, port, host):
		for comp in self.comps:
			if (comp.host.publicIP == host or comp.host.privateIP == host or comp.host.hostName == host) and comp.port == port:
				print 'returning the requested component with port:',port,' and host:',host
				return comp.idComp
		print 'No components to return port:',port,' and host:',host
		return ''


	#
	# getComponentPort
	#
	def getComponentPort(self, idComp, host):
		for comp in self.comps:
			if (comp.host.publicIP == host or comp.host.privateIP == host or comp.host.hostName == host) and comp.idComp == idComp:
				print 'returning the requested component with idComp:',idComp,' and host:',host
				return comp.port
		print 'No components to return idComp:',idComp,' and host:',host
		return 0

	#
	# getComponentHostByPort
	#	
	def getComponentHostNameByPort(self, port):
		for comp in self.comps:
			if comp.port == port:
				print 'returning the requested component with port:',port
				return comp.host.hostName
		print 'No components to return port:',port
		return ''

	#
	# getComponentHostById
	#
	def getComponentHostNameById(self, idComp):
		for comp in self.comps:
			if comp.idComp == idComp:
				print 'returning the requested component with port:',idComp
				return comp.host.hostName
		print 'No host for component:',idComp
		return ''