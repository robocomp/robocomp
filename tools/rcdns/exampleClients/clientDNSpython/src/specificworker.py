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



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'

		self.insertComps()
		##############################
		i=0
		while i<233:
			if i % 3 == 0:
				print self.searchIdComp(10000+i, 'robonuc0.local')
			i += 1
		##############################
		port0 = self.searchPort('comp0', 'robonuc0.local')
		port1 = self.searchPort('comp1', 'robonuc0.local')
		port2 = self.searchPort('noexist', 'robonuc0.local')		
		if port0 == 0:
			print 'no component up'
		else:
			print port0
		if port1 == 0:
			print 'no component up'
		else:
			print port1
		if port2 == 0:
			print 'no component up'
		else:
			print port2
		##############################
		search0 = self.searchHostnameById('comp33')
		search1 = self.searchHostnameById('comp32312')
		search2 = self.searchHostnameByPort(10221)
		search3 = self.searchHostnameByPort(929)
		if search0 == '':
			print 'no component up'
		else:
			print search0
		if search1 == '':
			print 'no component up'
		else:
			print search1
		if search2 == '':
			print 'no component up'
		else:
			print search2
		if search3 == '':
			print 'no component up'
		else:
			print search3			
		##############################

		print self.searchAllCompsHost('robonuc2.local')
		print self.searchAllComps()

		return True


###############################################
#			SERVANTS
###############################################
	def insertComps(self):
		for i in range(0,233):
			hostData = ipInfo()
			hostData.hostName = 'robonuc'+str(i%3)+'.local'
			hostData.publicIP = '158.49.123.'+str(i)
			hostData.privateIP = '192.168.1.'+str(i)
			if(self.rcdns_proxy.giveMePort('comp'+str(i),hostData) != 0):
				print 'ok giveMePort'
			else:
				print 'error in giveMePort'

	def searchIdComp(self, port, host):
		return self.rcdns_proxy.getComponentId(port, host)

	def searchPort(self, idComp, host):
		return self.rcdns_proxy.getComponentPort(idComp, host)

	def searchHostnameByPort(self, port):
		return self.rcdns_proxy.getComponentHostNameByPort(port)
	
	def searchHostnameById(self, idComp):
		return self.rcdns_proxy.getComponentHostNameById(idComp)

	def searchAllCompsHost(self, host):
		return self.rcdns_proxy.getAllCompsHost(host)

	def searchAllComps(self):
		return self.rcdns_proxy.getAllComps()