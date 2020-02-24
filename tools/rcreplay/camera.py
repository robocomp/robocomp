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

import Ice, threading
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from opencv.cv import *
from opencv.adaptors import *
import PIL
from ctypes import *

import RoboCompCamera
global RoboCompCamera


replay_plugin_identifier = 'camera_hal'


def getReplayClass():
	return CameraI()
def getRecordClass(proxy):
	return CameraRecorder(proxy)
def getGraphicalUserInterface():
	return CameraGUI()


class CameraGUI(QWidget):
	def __init__(self, parent=None):
		QWidget.__init__(self,parent)
		self.show()
		self.measure = None
		self.configuration = None
	def getSize(self):
		return QSize(self.configuration.width, self.configuration.height*self.configuration.numCams)
	def setConfiguration(self, configuration):
		self.configuration = configuration
	def setMeasure(self, measure):
		self.measure = measure
	def paintEvent(self, event):
		if self.measure and self.configuration:
			self.painter = QPainter(self)
			self.painter.setRenderHint(QPainter.Antialiasing, True)
			image = QImage(self.measure[0], self.configuration.width, self.configuration.height*self.configuration.numCams, QImage.Format_RGB888)
			self.painter.drawImage(QPointF(0, 0), image)
			self.painter = None


class CameraI(RoboCompCamera.Camera):
	def __init__(self):
		self.rgb = str()
		self.hState = str()
		self.bState = str()
		self.configuration = None
	def setConfiguration(self, configuration):
		self.configuration = configuration
		#self.rgbImage  = cvCreateImage(cvSize(self.configuration.width, 2*self.configuration.height), IPL_DEPTH_8U, 3)
		#self.grayImage = cvCreateImage(cvSize(self.configuration.width, 2*self.configuration.height), IPL_DEPTH_8U, 1)
		#self.rgbPlanes = cvCreateImage(cvSize(self.configuration.width, 2*self.configuration.height), IPL_DEPTH_8U, 3)
		#self.pilColor = PIL.Image.new("RGB", (self.configuration.width, 2*self.configuration.height), "Black")
	def setMeasure(self, measure):
		self.rgb = measure[0]
		self.pilColor = PIL.Image.frombuffer("RGB", (self.configuration.width, self.configuration.numCams*self.configuration.height), self.rgb, 'raw', 'RGB', 0, 1)
		self.grayImage = self.pilColor.convert("L")
		self.hState = measure[1]
		self.bState = measure[2]
	def getMeasure(self):
		return [self.rgb, self.hState, self.bState]
	# getYUVImage
	def getYUVImage(self, cam, current = None):
		return 'Not supported yet'
		#cvCvtColor(self.rgbImage, self.grayImage, CV_RGB2Luv)
		#if cam < self.configuration.numCams:
			#a = cam*self.configuration.width*self.configuration.height*2
			#b = (1+cam)*self.configuration.width*self.configuration.height*2
			#return ( self.grayImage[a:b], self.hState, self.bState )
		#elif cam == self.configuration.bothCameras:
			#return ( self.grayImage, self.hState, self.bState )
		#else:
			#return ( '', self.hState, self.bState )
	# getYImage
	def getYImage(self, cam, current = None):
		#cvCvtColor(self.rgbImage, self.grayImage, CV_RGB2GRAY)
		if cam < self.configuration.numCams:
			a = cam*self.configuration.width*self.configuration.height
			b = (1+cam)*self.configuration.width*self.configuration.height
			return ( self.grayImage.tostring()[a:b], self.hState, self.bState )
		elif cam == self.configuration.bothCameras:
			return ( self.grayImage.tostring(), self.hState, self.bState )
		else:
			return ( '', self.hState, self.bState )
	# getYLogPolarImage
	def getYLogPolarImage(self, cam, current = None):
		return 'Not yet implemented'
	# getYImageCR
	def getYImageCR(self, cam, current = None):
		return 'Not yet implemented'
	# getRGBPackedImage
	def getRGBPackedImage(self, cam, current = None):
		if cam < self.configuration.numCams:
			a = cam*self.configuration.width*self.configuration.height*3
			b = (1+cam)*self.configuration.width*self.configuration.height*3
			return ( self.rgb[a:b], self.hState, self.bState )
		elif cam == self.configuration.bothCameras:
			return ( self.rgb, self.hState, self.bState )
		else:
			return ( '', self.hState, self.bState )
	# getYRGBImage
	def getYRGBImage(self, cam, current = None):
		return 'Not yet implemented'
	# getCamParams
	def getCamParams(self, current = None):
		return self.configuration
	# setInnerImage
	def setInnerImage(self, roi):
		self.rgb = roi


class CameraRecorder:
	def __init__(self, proxy):
		global RoboCompCamera
		self.proxy = RoboCompCamera.CameraPrx.checkedCast(proxy)
	def getConfiguration(self):
		self.configuration = self.proxy.getCamParams()
		return self.configuration
	def getMeasure(self):
		if self.configuration.numCams>1:
			self.measure = self.proxy.getRGBPackedImage(self.configuration.bothCameras)
		else:
			self.measure = self.proxy.getRGBPackedImage(0)
		return self.measure
	def measure(self):
		return self.measure

