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
            pass

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True



    def compute(self):
        print('SpecificWorker.compute...')
        while True: pass
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
        import time
        time.sleep(0.2)
        exit()



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of addNewHand method from HandDetection interface
    #
    def HandDetection_addNewHand(self, expectedHands, roi):
        ret = int()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getHands method from HandDetection interface
    #
    def HandDetection_getHands(self):
        ret = RoboCompHandDetection.Hands()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getHandsCount method from HandDetection interface
    #
    def HandDetection_getHandsCount(self):
        ret = int()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompHandDetection you can use this types:
    # RoboCompHandDetection.TImage
    # RoboCompHandDetection.TRoi
    # RoboCompHandDetection.Hand


