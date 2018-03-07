#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import sys, traceback, Ice, IceStorm, subprocess, threading, time, Queue, os
import hashlib

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import QtGui, QtCore

import subprocess

from collections import namedtuple
Execution = namedtuple('Execution', ['path', 'binary', 'arguments', 'yakuakeTabName'])


class GenericWorker(QtCore.QObject):
	kill = QtCore.Signal()


	def __init__(self, mprx):
		super(GenericWorker, self).__init__()

		self.mutex = QtCore.QMutex()
		self.Period = 30
		self.timer = QtCore.QTimer(self)


	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print "Period changed", p
		Period = p
		timer.start(Period)

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Trying to read ~/.robocomp'
	try:
		lines = [x.strip() for x in open(os.getenv("HOME")+'/.robocomp', 'r').readlines() if len(x.strip())>0 and x[0] != '#']
	except:
		print 'can\'t read file'
		sys.exit()
	if len(lines) != 1:
		print 'empty?'
		sys.exit()
	if lines[0][0] != '/':
		print 'we need an absolute path in ~/.robocomp'
		sys.exit()
	ROBOCOMP = lines[0]
	print "Read $ROBOCOMP from ~/.robocomp <"+ROBOCOMP+">"
		
	


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"RCRemote.ice")
from RoboCompRemote import *


class RCRemoteI(RCRemote):
	def __init__(self, worker):
		self.worker = worker

	def run(self, stuff, hashedPassword, path, binary, arguments, yakuakeTabName, c):
		return self.worker.run(stuff, hashedPassword, path, binary, arguments, yakuakeTabName)




class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map, passwd):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.passwd = passwd
		self.onTheirWay = []

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	tracebak.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		#print 'SpecificWorker.compute...'
		#try:
		#	differentialrobot_proxy.setSpeed(100, 0)
		#except Ice.Exception, e:
		#	tracebak.print_exc()
		#	print e
		return True


	#
	# run
	#
	def run(self, stuff, hashedPassword, path, binary, arguments, yakuakeTabName):
		print 'BINARY', binary
		print 'PATH', path
		print 'TABNAME', yakuakeTabName
		print 'ARGS', arguments
	
		time.sleep(0.5)
		with QtCore.QMutexLocker(self.mutex) as locker:
			if hashedPassword != hashlib.sha224(stuff+self.passwd).hexdigest():
				print 'WRONG PASSWORD', hashedPassword
				return False
			else:
				if not yakuakeTabName in self.onTheirWay:
					self.onTheirWay.append(yakuakeTabName)
					p = subprocess.Popen(['/opt/robocomp/bin/rcremoteshell', binary, path, yakuakeTabName]+arguments)
					self.onTheirWay.remove(yakuakeTabName)
			time.sleep(0.5)
		return True



def getPassFor(host_param):
	for line in open(os.getenv("HOME")+'/.rcremote', 'r').readlines():
		s = line.strip().split('#', 1)
		if len(s) < 2:
			continue
		host = s[0]
		password = s[1]
		if host == host_param:
			return password
	raise Exception("can't find password for "+host_param+" in ~/.rcremote  (format 'host#password')")


if __name__ == '__main__':
	app = QtCore.QCoreApplication(sys.argv)
	ic = Ice.initialize(sys.argv)
	status = 0
	mprx = {}

	if len(sys.argv) < 2:
		try:
			print 'Reading password from ~/.rcremote...'
			password = getPassFor('localhost')
			print 'ok.'
		except:
			print 'Couldn\'t read password from file!'
			print '\nUsage: rcremoteserver <password>'
			sys.exit(-1)
	else:
		password = sys.argv[1]

	if status == 0:
		worker = SpecificWorker(mprx, password)

		adapter = ic.createObjectAdapterWithEndpoints('rcremote', 'tcp -p 4242')
		adapter.add(RCRemoteI(worker), ic.stringToIdentity('rcremote'))
		adapter.activate()
		app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
