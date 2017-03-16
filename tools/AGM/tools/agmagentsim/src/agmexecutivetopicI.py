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
	

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"

Ice.loadSlice(preStr+"AGMExecutive.ice")
from RoboCompAGMExecutive import *

class AGMExecutiveTopicI(AGMExecutiveTopic):
	def __init__(self, worker):
		self.worker = worker

	def structuralChange(self, w, c):
		return self.worker.structuralChange(w)
	def edgesUpdated(self, modifications, c):
		return self.worker.edgesUpdated(modifications)
	def edgeUpdated(self, modification, c):
		return self.worker.edgeUpdated(modification)
	def symbolUpdated(self, modification, c):
		return self.worker.symbolUpdated(modification)
	def symbolsUpdated(self, modifications, c):
		return self.worker.symbolsUpdated(modifications)





