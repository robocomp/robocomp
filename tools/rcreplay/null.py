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

import RoboCompSpeech
global RoboCompSpeech


replay_plugin_identifier = 'null'


def getReplayClass():
	return SpeechI()
def getRecordClass(proxy):
	return SpeechRecorder(proxy)
def getGraphicalUserInterface():
	return None


class SpeechI(RoboCompSpeech.Speech):
	def __init__(self):
		pass
	def setConfiguration(self, configuration):
		pass
	def setMeasure(self, measure):
		pass
	def getMeasure(self):
		return None
	def say(self, string, current = None):
		pass

class SpeechRecorder:
	def __init__(self, proxy):
		pass
	def getConfiguration(self):
		return None
	def getMeasure(self):
		return None
	def measure(self):
		return None

