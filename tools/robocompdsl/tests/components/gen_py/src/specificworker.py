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

import sys, os, traceback, time

from PySide import *
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

		self.substractBuffer = CallQueue()
		self.printmsgBuffer = CallQueue()
		self.sumBuffer = CallQueue()
		self.divideBuffer = CallQueue()


	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#computeCODE
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]

		# You can call the methods on proxy in blocking or non blocking way (async)
		# 
		# for calling in blocking way 
		# a, b = self.test_proxy.sum(100, 2)
		# 
		# async call using polling
		# 	call1 = test_proxy.begin_sum(100, 2)
		# poll using
		# 	call1.isCompleted()
		# get the return value
		# 	a,b = test_proxy.end_sum(call1)
		# 
		# async call uing callback
		# 	test_proxy.begin_sum(100, 2, lambda x:print(x), lambda ex:print(ex)
		# 


		# substract
		params, cid = self.substractBuffer.pop()
		if cid is not None:
			num1 = params["num1"]
			num2 = params["num2"]
			#
			#Logic for substract
			#
			result = num1 - num2
			self.substractBuffer.set_finished(cid, [result] )


		# printmsg
		params, cid = self.printmsgBuffer.pop()
		if cid is not None:
			message = params["message"]
			print(message)

		# sum
		params, cid = self.sumBuffer.pop()
		if cid is not None:
			num1 = params["num1"]
			num2 = params["num2"]
			ret = num2+num1
			self.sumBuffer.set_finished(cid, [ret])


		# divide
		params, cid = self.divideBuffer.pop()
		if cid is not None:
			divident = params["divident"]
			divisor = params["divisor"]
			ret = divident/divisor
			reminder = divident%divisor
			self.divideBuffer.set_finished(cid, [ret, reminder])

		return True

