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
import interfaces as ifaces

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
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
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
        print(f"Testing RoboCompCameraSimple.TImage from ifaces.RoboCompCameraSimple"
        test = ifaces.RoboCompCameraSimple.TImage()
        print(f"Testing RoboCompRGBD.ColorRGB from ifaces.RoboCompRGBD"
        test = ifaces.RoboCompRGBD.ColorRGB()
        print(f"Testing RoboCompRGBD.PointXYZ from ifaces.RoboCompRGBD"
        test = ifaces.RoboCompRGBD.PointXYZ()
        print(f"Testing RoboCompRGBD.CameraParameters from ifaces.RoboCompRGBD"
        test = ifaces.RoboCompRGBD.CameraParameters()
        print(f"Testing RoboCompRGBD.TRGBDParams from ifaces.RoboCompRGBD"
        test = ifaces.RoboCompRGBD.TRGBDParams()
        print(f"Testing RoboCompHandDetection.TImage from ifaces.RoboCompHandDetection"
        test = ifaces.RoboCompHandDetection.TImage()
        print(f"Testing RoboCompHandDetection.TRoi from ifaces.RoboCompHandDetection"
        test = ifaces.RoboCompHandDetection.TRoi()
        print(f"Testing RoboCompHandDetection.Hand from ifaces.RoboCompHandDetection"
        test = ifaces.RoboCompHandDetection.Hand()
        print(f"Testing RoboCompAprilTags.tag from ifaces.RoboCompAprilTags"
        test = ifaces.RoboCompAprilTags.tag()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Slots methods for State Machine ===================
    # ===================================================================

    #
    # sm_initialize
    #
    @QtCore.Slot()
    def sm_initialize(self):
        print("Entered state initialize")
        self.t_initialize_to_compute.emit()
        pass
    

    #
    # sm_compute
    #
    @QtCore.Slot()
    def sm_compute(self):
        print("Entered state compute")
        self.compute()
        pass


    #
    # sm_finalize
    #
    @QtCore.Slot()
    def sm_finalize(self):
        print("Entered state finalize")
        pass

    # =================================================================
    # =================================================================

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newAprilTag method from AprilTags interface
    #
    def AprilTags_newAprilTag(self, tags):
    
        #
        # write your CODE here
        #
        pass


    #
    # SUBSCRIPTION to newAprilTagAndPose method from AprilTags interface
    #
    def AprilTags_newAprilTagAndPose(self, tags, bState, hState):
    
        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================


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
        ret = ifaces.RoboCompHandDetection.Hands()
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
    # From the RoboCompCameraSimple you can call this methods:
    # self.camerasimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraSimple you can use this types:
    # RoboCompCameraSimple.TImage

    ######################
    # From the RoboCompRGBD you can call this methods:
    # self.rgbd_proxy.getData(...)
    # self.rgbd_proxy.getDepth(...)
    # self.rgbd_proxy.getDepthInIR(...)
    # self.rgbd_proxy.getImage(...)
    # self.rgbd_proxy.getRGB(...)
    # self.rgbd_proxy.getRGBDParams(...)
    # self.rgbd_proxy.getRegistration(...)
    # self.rgbd_proxy.getXYZ(...)
    # self.rgbd_proxy.getXYZByteStream(...)
    # self.rgbd_proxy.setRegistration(...)

    ######################
    # From the RoboCompRGBD you can use this types:
    # RoboCompRGBD.ColorRGB
    # RoboCompRGBD.PointXYZ
    # RoboCompRGBD.CameraParameters
    # RoboCompRGBD.TRGBDParams

    ######################
    # From the RoboCompAprilBasedLocalization you can publish calling this methods:
    # self.aprilbasedlocalization_proxy.newAprilBasedPose(...)

    ######################
    # From the RoboCompHandDetection you can use this types:
    # RoboCompHandDetection.TImage
    # RoboCompHandDetection.TRoi
    # RoboCompHandDetection.Hand

    ######################
    # From the RoboCompAprilTags you can use this types:
    # RoboCompAprilTags.tag


