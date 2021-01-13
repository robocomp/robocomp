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

import glob
import os
import sys
import time
from queue import Empty
from queue import Queue
import cv2
import numpy as np
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
import weakref
import math
from numpy import random
from carla import ColorConverter as cc

from genericworker import *

try:
    sys.path.append(glob.glob('/home/robocomp/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.load_world('CampusAvanz')
carla_map = world.get_map()
blueprint_library = world.get_blueprint_library()

blueprint = random.choice(blueprint_library.filter('vehicle.*'))
spawn_points = carla_map.get_spawn_points()
spawn_point = random.choice(spawn_points)
vehicle = world.spawn_actor(blueprint, spawn_point)
vehicle.set_autopilot(True)


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self.gnss_queue = Queue()
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._GNSS_callback(weak_self, event))

    @staticmethod
    def _GNSS_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        self.lat = sensor_data.latitude
        self.lon = sensor_data.longitude
        self.frame = sensor_data.frame
        self.timestamp = sensor_data.timestamp
        self.gnss_queue.put((self.timestamp, self.frame, self.lat, self.lon))

        # print('------------- GNSS -------------')
        # print('Latitud:', self.lat)
        # print('Longitud:', self.lon)
        # print('\n')


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.imu_queue = Queue()
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
        self.frame = sensor_data.frame
        self.timestamp = sensor_data.timestamp
        self.imu_queue.put((self.timestamp, self.frame, self.accelerometer, self.gyroscope, self.compass))


# # ==============================================================================
# # -- CameraManager -------------------------------------------------------------
# # ==============================================================================
#
#
class CameraManager(object):
    def __init__(self, bp_library, parent_actor, width, height):
        self.sensor = None
        self.blueprint_library = bp_library
        self.img_width = width
        self.img_height = height
        self._parent = parent_actor
        self.sensor_list = []
        self.cm_queue = Queue()
        world = self._parent.get_world()
        weak_self = weakref.ref(self)

        cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', f'{self.img_width}')
        cam_bp.set_attribute('image_size_y', f'{self.img_height}')
        cam_bp.set_attribute('fov', '110')
        cam_bp.set_attribute('sensor_tick', '1.0')

        lmanager = world.get_lightmanager()
        mylights = lmanager.get_all_lights()

        spawn_point_16 = carla.Transform(
            carla.Location(x=mylights[16].location.x, z=mylights[16].location.z + 5, y=mylights[16].location.y),
            carla.Rotation(pitch=-15, yaw=-90))
        cam01 = world.spawn_actor(cam_bp, spawn_point_16, attach_to=None)
        cam01.listen(lambda data: self.sensor_callback(weak_self, data, "camera RGB 01"))
        self.sensor_list.append(cam01)

        spawn_point_21 = carla.Transform(
            carla.Location(x=mylights[21].location.x, z=mylights[21].location.z + 5, y=mylights[21].location.y),
            carla.Rotation(pitch=-15, yaw=90))
        cam02 = world.spawn_actor(cam_bp, spawn_point_21, attach_to=None)
        cam02.listen(lambda data: self.sensor_callback(weak_self, data, "camera RGB 02"))
        self.sensor_list.append(cam02)

        spawn_point_5 = carla.Transform(
            carla.Location(x=mylights[5].location.x, z=mylights[5].location.z + 5, y=mylights[5].location.y),
            carla.Rotation(pitch=-15, yaw=-90))
        cam03 = world.spawn_actor(cam_bp, spawn_point_5, attach_to=None)
        cam03.listen(lambda data: self.sensor_callback(weak_self, data, "camera RGB 03"))
        self.sensor_list.append(cam03)

        spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
        cam04 = world.spawn_actor(cam_bp, spawn_point, attach_to=parent_actor)
        cam04.listen(lambda data: self.sensor_callback(weak_self, data, "camera RGB 04"))
        self.sensor_list.append(cam04)



    @staticmethod
    def sensor_callback(weak_self, img, sensor_name):
        self = weak_self()
        self.frame = img.frame
        self.timestamp = img.timestamp

        if not self:
            return
        if 'lidar' in sensor_name:
            points = np.frombuffer(img.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.width,self.height) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.width, 0.5 * self.height)
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

            self.cm_queue.put((img.frame, sensor_name, lidar_img))

        else:
            img.convert(cc.Raw)
            array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (img.height, img.width, 4))
            array = array[:, :, :3]
            # array = array[:, :, ::-1]
            self.cm_queue.put((self.timestamp, self.frame, sensor_name, array))

    def show_img(self, image, name):
        cv2.imshow(name, image)
        cv2.waitKey(1)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        global world, carla_map, blueprint_library, vehicle

        self.Period = 100
        self.img_width = 480
        self.img_height = 360

        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicle = vehicle

        self.gnss_sensor = GnssSensor(self.vehicle)
        self.imu_sensor = IMUSensor(self.vehicle)
        self.camera_manager = CameraManager(self.blueprint_library, self.vehicle, self.img_width, self.img_height)

        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

    def __del__(self):
        print('SpecificWorker destructor')
        cv2.destroyAllWindows()
        for sensor in self.sensor_list:
            sensor.destroy()

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')
        try:
            for i in range(0, len(self.camera_manager.sensor_list)):
                print('------------- RGB -------------')
                cm_timestamp, cm_frame, cm_sensor_name, cm_sensor_data = self.camera_manager.cm_queue.get(True, 1.0)
                self.camera_manager.show_img(cm_sensor_data, cm_sensor_name)
                print(f'TimeStamp: {cm_timestamp}   Frame: {cm_frame}   Sensor: {cm_sensor_name}     Shape:{cm_sensor_data.shape}')
                print('------------- GNSS -------------')
                gnss_timestamp, gnss_frame,  gnss_lat, gnss_lon = self.gnss_sensor.gnss_queue.get(True, 1.0)
                print(f'TimeStamp: {gnss_timestamp}   Frame: {gnss_frame}  Latitud: {gnss_lat} Longitud: {gnss_lon}' )
                print('------------- IMU -------------')
                imu_timestamp, imu_frame,  imu_accelerometer, imu_gyroscope, imu_compass = self.imu_sensor.imu_queue.get(True, 1.0)
                print(f'TimeStamp: {imu_timestamp}   Frame:{imu_frame}  Accelerometer: {imu_accelerometer}   ' 
                      f'Gyroscope:{imu_gyroscope} Compass: {imu_compass} ')

                print('\n')

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
        # print("Entered state compute")
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
