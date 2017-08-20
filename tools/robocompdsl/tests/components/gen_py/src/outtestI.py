#
# Copyright (C) 2017 by YOUR NAME HERE
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

additionalPathStr = ''
icePaths = []
try:
	icePaths.append('/opt/robocomp/interfaces')
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

ice_Test = False
for p in icePaths:
	print 'Trying', p, 'to load Test.ice'
	if os.path.isfile(p+'/Test.ice'):
		print 'Using', p, 'to load Test.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"Test.ice"
		Ice.loadSlice(wholeStr)
		ice_Test = True
		break
if not ice_Test:
	print 'Couln\'t load Test'
	sys.exit(-1)
from RoboCompTests import *

class outTestI(outTest):
	def __init__(self, worker):
		self.worker = worker

	def substract(self, num1, num2, c):
		return self.worker.substract(num1, num2)
	def printmsg(self, message, c):
		return self.worker.printmsg(message)
	def sum(self, num1, num2, c):
		return self.worker.sum(num1, num2)
	def divide(self, divident, divisor, c):
		return self.worker.divide(divident, divisor)
