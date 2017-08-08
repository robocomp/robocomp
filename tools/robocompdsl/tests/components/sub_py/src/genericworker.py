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

import sys, Ice, os
from Queue import Queue
from PySide import *


ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"Test.ice")
from RoboCompTests import *


from testI import *
from publishtestI import *


class GenericWorker(QtCore.QObject):
	kill = QtCore.Signal()


	def __init__(self, mprx):
		super(GenericWorker, self).__init__()




		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

		self.msgTestBuffer = CallQueue()
		self.printmsgBuffer = CallQueue()



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

	#
	# msgTest
	#
	def msgTest(self, id):
		kwargs = {'id': id}
		cid = self.msgTestBuffer.push(kwargs)
		while(self.msgTestBuffer.is_finished(cid)==False):
			pass
		return self.msgTestBuffer.result(cid)


	#
	# printmsg
	#
	def printmsg(self, message):
		# print 'function called', message
		kwargs = {'message': message}
		cid = self.printmsgBuffer.push(kwargs)
		while(self.printmsgBuffer.is_finished(cid)==False):
			pass
		return self.printmsgBuffer.result(cid)


class CallQueue(object):
	def __init__(self, maxsize=-1):
		self.mutex = QtCore.QMutex()
		self.call_buffer = Queue(maxsize=maxsize)
		self.current_id = 0
		self.result_buffer = dict()

	def push(self, params):
		"""
		psuh a call to queue

		:param params: params to call as a dictionary
		"""
		self.mutex.lock()
		self.call_buffer.put_nowait(params)
		self.current_id += 1
		self.mutex.unlock()
		print "new call added ", self.current_id
		return self.current_id

	def pop(self):
		cid = self.current_id
		self.mutex.lock()
		params = self.call_buffer.get_nowait()
		self.current_id -= 1
		self.mutex.unlock()
		print "call removed ", self.current_id
		return params, cid

	def result(self, cid):
		""" return result of an call """
		if self.is_finished(cid):
			result = self.result_buffer[cid]
			del self.result_buffer[cid]
			return result
		else:
			raise Exception("Id not Found:"+str(cid))

	def is_finished(self, cid):
		""" if a call has finished """
		return cid in self.result_buffer.keys()

	def set_finished(self, cid, result=None):
		""" set a call as finished"""
		self.result_buffer[cid] = result

	def empty(self):
		""" """
		return self.call_buffer.empty()