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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./src/ --all ./src/EmotionalMotor.ice")

from RoboCompEmotionalMotor import *

class EmotionalMotorI(EmotionalMotor):
    def __init__(self, worker):
        self.worker = worker


    def expressAnger(self, c):
        return self.worker.EmotionalMotor_expressAnger()

    def expressDisgust(self, c):
        return self.worker.EmotionalMotor_expressDisgust()

    def expressFear(self, c):
        return self.worker.EmotionalMotor_expressFear()

    def expressJoy(self, c):
        return self.worker.EmotionalMotor_expressJoy()

    def expressSadness(self, c):
        return self.worker.EmotionalMotor_expressSadness()

    def expressSurprise(self, c):
        return self.worker.EmotionalMotor_expressSurprise()

    def isanybodythere(self, isAny, c):
        return self.worker.EmotionalMotor_isanybodythere(isAny)

    def listening(self, setListening, c):
        return self.worker.EmotionalMotor_listening(setListening)

    def pupposition(self, x, y, c):
        return self.worker.EmotionalMotor_pupposition(x, y)

    def talking(self, setTalk, c):
        return self.worker.EmotionalMotor_talking(setTalk)
