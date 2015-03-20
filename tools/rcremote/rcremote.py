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

import sys, traceback, Ice, os

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL) # Ctrl+c handling

from PySide import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"RCRemote.ice")
import RoboCompRemote


if __name__ == '__main__':
	app = QtCore.QCoreApplication(sys.argv)
	ic = Ice.initialize(sys.argv)
	status = 0
	mprx = {}
	try:
		try:
			proxyString = "rcremote:tcp -p 4242 -h " + sys.argv[1]
			try:
				basePrx = ic.stringToProxy(proxyString)
				rcremote_proxy = RoboCompRemote.RCRemotePrx.checkedCast(basePrx)
				mprx["RCRemoteProxy"] = rcremote_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (RCRemote)', proxyString
				#traceback.print_exc()
				status = 1
		except Ice.Exception, e:
			print e
			print 'Cannot get RCRemoteProxy property.'
			status = 1

	except:
			traceback.print_exc()
			status = 1


	if status == 0:
		password = 'pass'
		path = '/'
		binary = 'dede'
		arguments = ['a', 'b', 'c']
		yakuakeTabName = 'tabname'
		if rcremote_proxy.run(password, path, binary, arguments, yakuakeTabName):
			print 'ok'
			sys.exit(0)
		else:
			print 'error'
			sys.exit(-1)
	else:
		print 'probleemm'
		sys.exit(-1)

