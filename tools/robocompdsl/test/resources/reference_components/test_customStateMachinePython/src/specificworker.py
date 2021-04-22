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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.myStateMachine.start()

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Slots methods for State Machine ===================
    # ===================================================================

    #
    # sm_one
    #
    @QtCore.Slot()
    def sm_one(self):
        print("Entered state one")
        pass

    #
    # sm_four
    #
    @QtCore.Slot()
    def sm_four(self):
        print("Entered state four")
        pass

    #
    # sm_three
    #
    @QtCore.Slot()
    def sm_three(self):
        print("Entered state three")
        pass

    #
    # sm_two
    #
    @QtCore.Slot()
    def sm_two(self):
        print("Entered state two")
        pass

    #
    # sm_five
    #
    @QtCore.Slot()
    def sm_five(self):
        print("Entered state five")
        pass

    # =================================================================
    # =================================================================




