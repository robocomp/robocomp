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
import glob
import os
import sys
from queue import Empty
from queue import Queue
import cv2
import numpy as np

try:
    sys.path.append(glob.glob('/home/robocomp/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.img_width = 480
        self.img_height = 360

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        #TODO Read client world form config
        self.world = client.load_world('CampusAvanz')
        self.blueprint_library = self.world.get_blueprint_library()

        self.sensor_list = []
        self.sensor_queue = Queue()

        self.initializeSensors()


        if startup_check:
            self.startup_check()
        else:
            self.Period = 0
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

    def initializeSensors(self):
        cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', f'{self.img_width}')
        cam_bp.set_attribute('image_size_y', f'{self.img_height}')
        cam_bp.set_attribute('fov', '110')
        cam_bp.set_attribute('sensor_tick', '100.0')

        lmanager = self.world.get_lightmanager()
        mylights = lmanager.get_all_lights()

        spawn_point_16 = carla.Transform(
            carla.Location(x=mylights[16].location.x, z=mylights[16].location.z + 5, y=mylights[16].location.y),
            carla.Rotation(pitch=-15, yaw=-90))
        cam01 = self.world.spawn_actor(cam_bp, spawn_point_16, attach_to=None)
        self.sensor_list.append(cam01)

        spawn_point_21 = carla.Transform(
            carla.Location(x=mylights[21].location.x, z=mylights[21].location.z + 5, y=mylights[21].location.y),
            carla.Rotation(pitch=-15, yaw=90))
        cam02 = self.world.spawn_actor(cam_bp, spawn_point_21, attach_to=None)
        self.sensor_list.append(cam02)

        spawn_point_5 = carla.Transform(
            carla.Location(x=mylights[5].location.x, z=mylights[5].location.z + 5, y=mylights[5].location.y),
            carla.Rotation(pitch=-15, yaw=-90))
        cam03 = self.world.spawn_actor(cam_bp, spawn_point_5, attach_to=None)
        self.sensor_list.append(cam03)

        cam01.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera01"))
        cam02.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera02"))
        cam03.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera03"))

    def __del__(self):
        print('SpecificWorker destructor')
        cv2.destroyAllWindows()
        for sensor in self.sensor_list:
            sensor.destroy()

    def setParams(self, params):
        return True

    def sensor_callback(self, image, sensor_queue, sensor_name):
        i = np.array(image.raw_data)
        i2 = i.reshape((self.img_height, self.img_width, 4))
        i3 = i2[:, :, :3]
        sensor_queue.put((image.frame, sensor_name, i3))

    def process_img(self, image, name):
        cv2.imshow(name, image)
        cv2.waitKey(1)

    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        try:
            for i in range(0, len(self.sensor_list)):
                s_frame = self.sensor_queue.get(True, 1.0)
                print(f'Frame: {s_frame[0]}   Sensor: {s_frame[1]}     Shape:{s_frame[2].shape}')
                self.process_img(s_frame[2], s_frame[1])

        except Empty:
            print('No data found')
        return True

    def startup_check(self):
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
