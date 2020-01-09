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

import Ice, sys, math, traceback

from PySide.QtCore import *
from PySide.QtGui import *
from ui_kinectDlg import Ui_KinectDlg
import numpy as np
import cv2


class C(QWidget):
	def __init__(self, endpoint, modules):
		QWidget.__init__(self)
		self.ui = Ui_KinectDlg()
		self.ui.setupUi(self)
		#hide kinect interface options
		self.ui.sbTilt.hide()
		self.ui.pbSetLed.hide()
		self.ui.label_4.hide()
		self.ui.cbLedOpt.hide()

		self.t = 0.
		arg = sys.argv
		arg.append("--Ice.MessageSizeMax=2000000")
		self.ic = Ice.initialize(arg)
		self.mods = modules
		self.prx = self.ic.stringToProxy(endpoint)
		self.proxy = self.mods['RoboCompCameraRGBDSimple'].CameraRGBDSimplePrx.checkedCast(self.prx)
		self.show()
		
		self.maxDepth = 90000
		self.method_combo = ""
		self.job()

	def job(self):
		self.method_combo = unicode(self.ui.method_combobox.currentText())
		if "getData" in self.method_combo:
			try:
				self.color, self.depth = self.proxy.getAll()
				if (len(self.color.image) == 0) or (len(self.depth.depth) == 0):
					print 'Error retrieving images!'
			except Ice.Exception:
				traceback.print_exc()
		elif "getImage" in self.method_combo:
			try:
				self.color = self.proxy.getImage()
				if (len(self.color.image) == 0):
					print 'Error retrieving images!'
			except Ice.Exception:
				traceback.print_exc()
		elif "getDepth" in self.method_combo:
			try:
				self.depth = self.proxy.getDepth()
				if (len(self.depth.depth) == 0):
					print 'Error retrieving images!'
			except Ice.Exception:
				traceback.print_exc()

	def paintEvent(self, event=None):
		color_image_height = 0
		color_image_width = 0
		depth_image_height = 0
		depth_image_width = 0
		# print "Paint event "+str(len(self.depth))+" "+str(type(self.depth))
		if "getData" in self.method_combo:
			depth_image_width = self.depth.width
			depth_image_height = self.depth.height
			color_image_width = self.color.width
			color_image_height = self.color.height
		if "getDepth" in self.method_combo:
			depth_image_width = self.depth.width
			depth_image_height = self.depth.height
		if "getImage" in self.method_combo:
			color_image_width = self.color.width
			color_image_height = self.color.height
			
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)

		if depth_image_width!= 0 and depth_image_height!=0:
			depth = np.array(self.depth.depth, dtype=np.float32)
			depth_min = np.min(depth)
			depth_max = np.max(depth)
			if depth_max != depth_min and depth_max > 0:
				depth = np.interp(depth, [depth_min, depth_max], [255.0, 0.0], right=0.0, left=0.0)
			depth = depth.astype(np.uint8)

			# Reshape to grayscale matrix
			depth = depth.reshape(depth_image_height, depth_image_width)

			# Convert to
			if depth.dtype == np.uint8:
				if len(depth.shape) == 2:
					gray_color_table = [qRgb(i, i, i) for i in range(256)]
					imageGrey = QImage(depth.data, depth.shape[1], depth.shape[0], depth.strides[0], QImage.Format_Indexed8)
					imageGrey.setColorTable(gray_color_table)
				else:
					print("Wrong depth matrix format: Shape %s"%(depth.shape))
			painter.drawImage(QPointF(self.ui.frame.x(), self.ui.frame.y()), imageGrey)
		#print 'mean', float(m)/t
		if color_image_width!= 0 and color_image_height!= 0:
			image = QImage(self.color.image, color_image_width, color_image_height, QImage.Format_RGB888)
			#image.save("images/image"+str(self.lalala)+'.png')
			painter.drawImage(QPointF(self.ui.frameRGB.x(), self.ui.frameRGB.y()), image)

		painter.end()
		#painter = None

