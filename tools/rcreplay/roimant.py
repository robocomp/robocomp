# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab
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

import Ice, threading, traceback, math
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *
from opencv.cv import *
from opencv.adaptors import *
import PIL
from ctypes import *

import RoboCompRoimant
global RoboCompRoimant


replay_plugin_identifier = 'roimant_hal'


def getReplayClass():
	return RoimantI()
def getRecordClass(proxy):
	return RoimantRecorder(proxy)
def getGraphicalUserInterface():
	return RoimantGUI()


class RoimantGUI(QWidget):
	def __init__(self, parent=None):
		QWidget.__init__(self,parent)
		self.show()
		self.measure = None
		self.configuration = None
		self.mutex = QMutex()
	def setConfiguration(self, configuration):
		self.configuration = configuration
		self.wdth = self.configuration[1].width
		self.hght = self.configuration[1].height
	def getSize(self):
		return QSize(self.wdth*2, self.hght*2)
	def setMeasure(self, measure):
		self.mutex.lock()
		self.measure = measure
		self.mutex.unlock()
	def paintEvent(self, event):
		event.accept()
		self.mutex.lock()
		if self.measure:
			painter = QPainter(self)
			painter.setRenderHint(QPainter.Antialiasing, True)
			painter.setPen(Qt.red)
			try:
				ui_offset_x = 0
				ui_offset_y = 0
				img_offset = 0
				for i in range(4):
					img_width = self.wdth/(2**i)
					img_height = self.hght/(2**i)
					img_size = img_width*img_height*3
					image = QImage(self.measure[0][i], img_width, img_height, QImage.Format_RGB888)
					painter.drawImage(QPointF(ui_offset_x, ui_offset_y), image)
					for r in self.measure[1]:
						if r.casado and r.level==i:
							painter.drawRect(ui_offset_x+r.xBase/r.scale-2, ui_offset_y+r.yBase/r.scale-2, 5, 5)
							painter.drawLine(ui_offset_x+r.xBase/r.scale, ui_offset_y+r.yBase/r.scale, ui_offset_x+self.wdth+r.xHomo/r.scale, ui_offset_y+r.yHomo/r.scale)
					ui_offset_x += img_width/4
					ui_offset_y += img_height
					img_offset += img_size

				ui_offset_x = self.wdth
				ui_offset_y = 0
				img_offset = 0
				for i in range(4):
					img_width = self.wdth/(2**i)
					img_height = self.hght/(2**i)
					img_size = img_width*img_height*3
					image = QImage(self.measure[2][i], img_width, img_height, QImage.Format_RGB888)
					painter.drawImage(QPointF(ui_offset_x, ui_offset_y), image)
					for r in self.measure[1]:
						if r.casado and r.level==i:
							painter.drawRect(ui_offset_x+r.xHomo/r.scale-2, ui_offset_y+r.yHomo/r.scale-2, 5, 5)
							painter.drawLine(ui_offset_x+r.xBase/r.scale-self.wdth, ui_offset_y+r.yBase/r.scale, ui_offset_x+r.xHomo/r.scale, ui_offset_y+r.yHomo/r.scale)
					ui_offset_x += img_width/4
					ui_offset_y += img_height
					img_offset += img_size
			except:
				traceback.print_exc()
			painter.end()
			painter = None
		self.mutex.unlock()


class RoimantI(RoboCompRoimant.Roimant):
	def __init__(self):
		self.leftPyr = str()
		self.rightPyr = str()
		self.tState = str()
		self.roiList = None
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def getMeasure(self):
		return self.measure
	# [ pirLeft        roiListLeft        pirRight        state ]
	def getROIList(self, current = None):
		return self.measure[1], self.measure[3]
	def getROIListAndVergencePyr(self, current = None):
		return self.measure[1], self.measure[0], self.measure[3]
	def getBothPyramidsAndLeftROIList(self, current = None):
		return self.measure
	def getBothPyramidsRGBAndLeftROIList(self, current = None):
		return self.measure
	def setROIListDescriptors(self, roiList, current = None):
		pass
	def getCamParams(self, current = None):
		return self.configuration[0]
	def getRoiParams(self, current = None):
		return self.configuration[1]

class RoimantRecorder:
	def __init__(self, proxy):
		global RoboCompRoimant
		print proxy
		self.proxy = RoboCompRoimant.RoimantPrx.checkedCast(proxy)
	def getConfiguration(self):
		self.configuration = self.proxy.getCamParams(), self.proxy.getRoiParams()
		return self.configuration
	def getMeasure(self):
		self.measure = self.proxy.getBothPyramidsRGBAndLeftROIList()
		return self.measure
	def measure(self):
		return self.measure

