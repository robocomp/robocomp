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
from Queue import Queue, Empty
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


from outtestI import *


class GenericWorker(QtCore.QObject):
	kill = QtCore.Signal()


	def __init__(self, mprx):
		super(GenericWorker, self).__init__()




		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
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

	# divide
	def divide(self, divident,divisor):
		kwargs = {"divident":divident,"divisor":divisor,}
		cid = self.divideBuffer.push(kwargs)
		while(self.divideBuffer.is_finished(cid)==False): pass
		return tuple(self.divideBuffer.result(cid))


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
		self.call_buffer.put_nowait((self.current_id, params))
		cid = self.current_id
		self.current_id += 1
		self.mutex.unlock()
		print "new params added, id: ", cid
		return cid

	def pop(self):
		try:
			self.mutex.lock()
			cid, params = self.call_buffer.get_nowait()
			self.mutex.unlock()
			print "params popped, id: ", cid
			return params, cid
		except Empty:
			self.mutex.unlock()
			return None, None

	def result(self, cid):
		""" return result of an call """
		if self.is_finished(cid):
			result = self.result_buffer[cid]
			# print "result popped, id: ", cid
			del self.result_buffer[cid]
			return result
		else:
			raise Exception("Id not Found:"+str(cid))

	def is_finished(self, cid):
		""" if a call has finished """
		return cid in self.result_buffer.keys()

	def set_finished(self, cid, result=None):
		""" set a call as finished"""
		# print "result added, id: ", cid
		self.result_buffer[cid] = result

	def empty(self):
		""" """
		return self.call_buffer.empty()

