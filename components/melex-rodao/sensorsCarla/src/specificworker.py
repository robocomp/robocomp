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
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

        # print('------------- GNSS -------------')
        # print('Latitud:', self.lat)
        # print('Longitud:', self.lon)
        # print('\n')


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
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

        # print('------------- IMU -------------')
        # print('Accelerometer:', self.accelerometer)
        # print('Gyroscope:', self.gyroscope)
        # print('Compass:', self.compass)
        # print('\n')


# # ==============================================================================
# # -- CameraManager -------------------------------------------------------------
# # ==============================================================================
#
#
# class CameraManager(object):
#     def __init__(self, parent_actor, width, height):
#         self.sensor = None
#         self.width = width
#         self.height = height
#         self._parent = parent_actor
#         self.sensor_list = []
#         self.sensor_queue = Queue()
#
#         lmanager = self.world.get_lightmanager()
#         mylights = lmanager.get_all_lights()
#
#         self.sensors = [
#             ['sensor.camera.rgb', cc.Raw, 'Camera RGB 01', carla.Transform(
#             carla.Location(x=mylights[16].location.x, z=mylights[16].location.z + 5, y=mylights[16].location.y),
#             carla.Rotation(pitch=-15, yaw=-90)), {}, None],
#             ['sensor.camera.rgb', cc.Raw, 'Camera RGB 02',  carla.Transform(
#             carla.Location(x=mylights[21].location.x, z=mylights[21].location.z + 5, y=mylights[21].location.y),
#             carla.Rotation(pitch=-15, yaw=90)), {}, None],
#             ['sensor.camera.rgb', cc.Raw, 'Camera RGB 03', carla.Transform(
#             carla.Location(x=mylights[5].location.x, z=mylights[5].location.z + 5, y=mylights[5].location.y),
#             carla.Rotation(pitch=-15, yaw=-90)) , {}, None],
#             ['sensor.camera.rgb', cc.Raw, 'Camera RGB 04', carla.Transform(carla.Location(x=2.5, z=0.7)), {}, parent_actor],
#             # ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
#             # ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
#             # ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
#             # ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}]
#         ]
#
#         world = self._parent.get_world()
#         bp_library = world.get_blueprint_library()
#         for item in self.sensors:
#             bp_name, color_converter, sensor_name, sp, attrs, attach = item
#             bp = bp_library.find(bp_name)
#             if bp_name.startswith('sensor.camera'):
#                 bp.set_attribute('image_size_x', f'{self.width}')
#                 bp.set_attribute('image_size_y', f'{self.height}')
#                 bp.set_attribute('fov', '110')
#                 bp.set_attribute('sensor_tick', '1.0')
#                 # if bp.has_attribute('gamma'):
#                 #     bp.set_attribute('gamma', str(gamma_correction))
#                 for attr_name, attr_value in attrs.items():
#                     bp.set_attribute(attr_name, attr_value)
#             elif bp_name.startswith('sensor.lidar'):
#                 self.lidar_range = 50
#                 for attr_name, attr_value in attrs.items():
#                     bp.set_attribute(attr_name, attr_value)
#                     if attr_name == 'range':
#                         self.lidar_range = float(attr_value)
#
#             sensor = world.spawn_actor(bp, sp, attach_to=attach)
#             sensor.listen(lambda data: self.sensor_callback(data, self.sensor_queue, sensor_name))
#             self.sensor_list.append(sensor)
#             # item.append(bp)
#
#     @staticmethod
#     def _parse_image(weak_self, image):
#         self = weak_self()
#         if not self:
#             return
#         if self.sensors[self.index][0].startswith('sensor.lidar'):
#             points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
#             points = np.reshape(points, (int(points.shape[0] / 4), 4))
#             lidar_data = np.array(points[:, :2])
#             lidar_data *= min(self.width,self.height) / (2.0 * self.lidar_range)
#             lidar_data += (0.5 * self.width, 0.5 * self.height)
#             lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
#             lidar_data = lidar_data.astype(np.int32)
#             lidar_data = np.reshape(lidar_data, (-1, 2))
#             lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
#             lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
#             lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
#
#             self.sensor_queue.put((image.frame, sensor_name, i3))
#
#         else:
#             image.convert(self.sensors[self.index][1])
#             array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#             array = np.reshape(array, (image.height, image.width, 4))
#             array = array[:, :, :3]
#             array = array[:, :, ::-1]
#             self.sensor_queue.put((image.frame, sensor_name, array))
#


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        global world, carla_map, blueprint_library, vehicle

        self.Period = 0
        self.img_width = 480
        self.img_height = 360

        self.world = world
        self.blueprint_library = blueprint_library
        self.vehicle = vehicle

        self.gnss_sensor = GnssSensor(self.vehicle)
        self.imu_sensor = IMUSensor(self.vehicle)
        # self.camera_manager = CameraManager(self.vehicle, self.img_width, self.img_height)

        self.sensor_list = []
        self.sensor_queue = Queue()
        self.initializeSensors()

        if startup_check:
            self.startup_check()
        else:
            self.timer.start(self.Period)
            self.defaultMachine.start()
            self.destroyed.connect(self.t_compute_to_finalize)

    # def get_random_vehicle(self, carla_map):
    #     blueprint = random.choice(self.blueprint_library.filter('vehicle.*'))
    #     spawn_points = carla_map.get_spawn_points()
    #     spawn_point = random.choice(spawn_points)
    #     self.vehicle = self.world.spawn_actor(blueprint, spawn_point)
    #     self.vehicle.set_autopilot(True)

    def initializeSensors(self):
        cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', f'{self.img_width}')
        cam_bp.set_attribute('image_size_y', f'{self.img_height}')
        cam_bp.set_attribute('fov', '110')
        cam_bp.set_attribute('sensor_tick', '1.0')

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

        spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
        cam04 = self.world.spawn_actor(cam_bp, spawn_point, attach_to=self.vehicle)
        self.sensor_list.append(cam04)

        cam01.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera01"))
        cam02.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera02"))
        cam03.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera03"))
        cam04.listen(lambda data: self.sensor_callback(data, self.sensor_queue, "camera04"))

    def __del__(self):
        print('SpecificWorker destructor')
        cv2.destroyAllWindows()
        for sensor in self.sensor_list:
            sensor.destroy()

    def setParams(self, params):
        return True

    def sensor_callback(self, image, sensor_queue, sensor_name):
        print('sensor_callback',sensor_name)
        i = np.array(image.raw_data)
        i2 = i.reshape((self.img_height, self.img_width, 4))
        i3 = i2[:, :, :3]
        sensor_queue.put((image.frame, sensor_name, i3))

    def process_img(self, image, name):
        cv2.imshow(name, image)
        cv2.waitKey(1)

    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')
        try:
            for i in range(0, len(self.sensor_list)):
                sensor_frame, sensor_name, sensor_data = self.sensor_queue.get(True, 1.0)
                print(f'Frame: {sensor_frame}   Sensor: {sensor_name}     Shape:{sensor_data.shape}')
                self.process_img(sensor_data, sensor_name)

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
