# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab - University of Extremadura
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


import inspect, time, sys, os, Ice

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

Ice.loadSlice(ROBOCOMP+"/interfaces/Logger.ice")
import RoboCompLogger



class qlog():
	def __init__(self, _loggerPrx, logger_mode):
		self.logger_mode = logger_mode
		self.loggerPrx = _loggerPrx
	def message(self, msg, _type):
		m = RoboCompLogger.LogMessage()
		m.message = msg
		m.file = str(inspect.stack()[2][0].f_code.co_filename).split('/')[-1]
		m.method = inspect.stack()[2][0].f_code.co_name
		m.line =  inspect.stack()[2][0].f_lineno
		m.timeStamp = time.strftime("%Y.%m.%d %H:%M:%S:",time.localtime())
		m.timeStamp += str(time.time()).split('.')[1]
		m.type = _type
		m.fullpath = os.path.abspath(inspect.stack()[2][0].f_code.co_filename)
      
		sender = m.fullpath.split('/')
		i = 0
		while (sender[i]!="robocomp"):
			i += 1
		if(sender[i+1]=="classes"):
			sender=sender[i+2]
		else:
			if(sender[i+2]=="HAL"):
				sender = sender[i+3]
			else:
				sender = sender[i+4]
		m.sender = sender
		self.Log = m

	def messageToconsole(self):
		print self.Log.timeStamp+"::"+self.Log.type+"::"+self.Log.sender+"::"+self.Log.message
	def messageTologger(self):
		self.loggerPrx.sendMessage(self.Log)
	def send(self,m,_type):
		self.message(m,_type)
		if(self.logger_mode=="local" or self.logger_mode=="both"):
			self.messageToconsole();
		if(self.logger_mode=="logger" or self.logger_mode=="both"):
			self.messageTologger();
		    
