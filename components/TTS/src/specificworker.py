#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

import subprocess
import sys
from google_speech import Speech
# import pyttsx3
sys.path.append("../")
try:
	from Queue import Queue
except ImportError:
	from queue import Queue

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *

max_queue = 100
charsToAvoid = ["'", '"', '{', '}', '[', '<', '>', '(', ')', '&', '$', '|', '#']

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.audioenviado = False
        self.text_queue = Queue(max_queue)
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        if "tts" in params:
            self._tts = params["tts"]
        else:
            self._tts = "festival"
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True


    @QtCore.Slot()
    def compute(self):
        if self.text_queue.empty():
            pass
        else:
            text_to_say = self.text_queue.get()
            for rep in charsToAvoid:
                text_to_say = text_to_say.replace(rep, '\\' + rep)
            #				shellcommand = "echo " + text_to_say  + " | padsp festival --tts"
            self.habla(text_to_say)

    def habla(self, text):
        try:
            print(text)
            self.emotionalmotor_proxy.talking(True)            
            # self.recorder_proxy.recording(False)
            self.pyttshtts(text)
            # self.recorder_proxy.recording(True)
            self.emotionalmotor_proxy.talking(False)
        except ImportError:
            print("Problema say with googgle")
            print("\033[91m To use google TTS you need to install gTTS package and playsound\033[00m")
            print("\033[91m You can try to install it with pip install gTTS playsound\033[00m")

    def Speech_say(self, text, owerwrite):
        if owerwrite:
            self.text_queue = Queue(max_queue)
        self.text_queue.put(text)
        return True

    def pyttshtts(self, text):
        lang = "es"
        speech = Speech(text, lang)
        # speech.play()

        # you can also apply audio effects while playing (using SoX)
        # see http://sox.sourceforge.net/sox.html#EFFECTS for full effect documentation
        effects = ("pitch", str(self.pitch), "tempo", str(self.tempo))
        # sox_effects2 = ("tempo", "0.5")
        speech.play(effects)
        # speech.play(sox_effects2)

    def Speech_setPitch(self, pitch):
        # self.pitch = 1
        self.pitch = pitch*4
        print(self.pitch)

    def Speech_setTempo(self, tempo):
        # self.tempo = 1
        self.tempo = tempo/10
        print(self.tempo)

