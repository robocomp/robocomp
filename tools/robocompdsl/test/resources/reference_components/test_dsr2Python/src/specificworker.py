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
from genericworker import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
from pydsr import *

# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

def update_node_att(id: int, attribute_names: [str]):
    print("UPDATE NODE ATT: ", id, " ", attribute_names)

def update_node(id: int, type: str):
    print("UPDATE NODE: ", id," ",  type)

def delete_node(id: int):
    print("DELETE NODE: ", id)

def update_edge(fr: int, to: int, type : str):
    print("UPDATE EDGE: ", fr," ", to," ", type)

def update_edge_att(fr: int, to: int, attribute_names : [str]):
    print("UPDATE EDGE ATT: ", fr," ", to," ", attribute_names)

def delete_edge(fr: int, to: int, type : str):
    print("DELETE EDGE: ", fr," ", to," ", type)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000

        self.g = DSRGraph(0, "pythonAgent", 111)

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, update_node)
            signals.connect(self.g, signals.DELETE_NODE, delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, delete_edge)
            print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)




