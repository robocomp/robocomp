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

import sys, traceback, Ice, os, copy

import signal, hashlib
signal.signal(signal.SIGINT, signal.SIG_DFL) # Ctrl+c handling

from PySide import *

import string
import random
def random_stuff_generator(size=16, chars=string.ascii_uppercase + string.digits):
	return ''.join(random.choice(chars) for _ in range(size))


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

def getPassFor(host_param):
	for line in open(os.getenv("HOME")+'/.rcremote', 'r').readlines():
		s = line.strip().split('#', 1)
		if len(s) < 2:
			continue
		host = s[0]
		password = s[1]
		if host == host_param:
			return password
	raise Exception("can't find password for "+host+" in ~/.rcremote  (format 'host#password')")


if len(sys.argv) < 5:
	print "EXAMPLE: rcremote localhost mytabname /home/robocomp touch argument1 argument2"
	sys.exit(-1)

if __name__ == '__main__':
	argv = copy.deepcopy(sys.argv)
	app = QtCore.QCoreApplication(sys.argv)
	ic = Ice.initialize(['me'])
	status = 0
	mprx = {}
	try:
		try:
			proxyString = "rcremote:tcp -p 4242 -h " + argv[1]
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
		print argv
		host = argv[1]
		yakuakeTabName = argv[2]
		path = argv[3]
		binary = argv[4]
		arguments = argv[5:]
		print 'path', path
		print 'binary', binary
		print 'arguments', arguments
		randomStuff = random_stuff_generator()
		password = getPassFor(host)
		hashedPassword = hashlib.sha224(randomStuff+password).hexdigest()
		if rcremote_proxy.run(randomStuff, hashedPassword, path, binary, arguments, yakuakeTabName):
			print 'ok'
			sys.exit(0)
		else:
			print 'error'
			sys.exit(-1)
	else:
		print 'probleemm'
		sys.exit(-1)

