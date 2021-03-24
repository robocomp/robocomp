import os
import pathlib
import time
import traceback
import Ice
import IceStorm
from rich.console import Console, Text
console = Console()


Ice.loadSlice("-I ./src/ --all ./src/AprilBasedLocalization.ice")
import RoboCompAprilBasedLocalization
Ice.loadSlice("-I ./src/ --all ./src/AprilTags.ice")
import RoboCompAprilTags
Ice.loadSlice("-I ./src/ --all ./src/CameraSimple.ice")
import RoboCompCameraSimple
Ice.loadSlice("-I ./src/ --all ./src/GenericBase.ice")
import RoboCompGenericBase
Ice.loadSlice("-I ./src/ --all ./src/HandDetection.ice")
import RoboCompHandDetection
Ice.loadSlice("-I ./src/ --all ./src/JointMotor.ice")
import RoboCompJointMotor
Ice.loadSlice("-I ./src/ --all ./src/RGBD.ice")
import RoboCompRGBD

class tagsList(list):
    def __init__(self, iterable=list()):
        super(tagsList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompAprilTags.tag)
        super(tagsList, self).insert(index, item)

setattr(RoboCompAprilTags, "tagsList", tagsList)
class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompCameraSimple, "ImgType", ImgType)
class ImgType(list):
    def __init__(self, iterable=list()):
        super(ImgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(ImgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(ImgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(ImgType, self).insert(index, item)

setattr(RoboCompHandDetection, "ImgType", ImgType)
class KeyPoint(list):
    def __init__(self, iterable=list()):
        super(KeyPoint, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, int)
        super(KeyPoint, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, int)
        super(KeyPoint, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, int)
        super(KeyPoint, self).insert(index, item)

setattr(RoboCompHandDetection, "KeyPoint", KeyPoint)
class TCoordSequence(list):
    def __init__(self, iterable=list()):
        super(TCoordSequence, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.KeyPoint)
        super(TCoordSequence, self).insert(index, item)

setattr(RoboCompHandDetection, "TCoordSequence", TCoordSequence)
class TContour(list):
    def __init__(self, iterable=list()):
        super(TContour, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.TCoordSequence)
        super(TContour, self).insert(index, item)

setattr(RoboCompHandDetection, "TContour", TContour)
class Hands(list):
    def __init__(self, iterable=list()):
        super(Hands, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompHandDetection.Hand)
        super(Hands, self).insert(index, item)

setattr(RoboCompHandDetection, "Hands", Hands)
class MotorParamsList(list):
    def __init__(self, iterable=list()):
        super(MotorParamsList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorParams)
        super(MotorParamsList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorParamsList", MotorParamsList)
class MotorGoalPositionList(list):
    def __init__(self, iterable=list()):
        super(MotorGoalPositionList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalPosition)
        super(MotorGoalPositionList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorGoalPositionList", MotorGoalPositionList)
class MotorGoalVelocityList(list):
    def __init__(self, iterable=list()):
        super(MotorGoalVelocityList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompJointMotor.MotorGoalVelocity)
        super(MotorGoalVelocityList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorGoalVelocityList", MotorGoalVelocityList)
class MotorList(list):
    def __init__(self, iterable=list()):
        super(MotorList, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, str)
        super(MotorList, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, str)
        super(MotorList, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, str)
        super(MotorList, self).insert(index, item)

setattr(RoboCompJointMotor, "MotorList", MotorList)
class imgType(list):
    def __init__(self, iterable=list()):
        super(imgType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, byte)
        super(imgType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, byte)
        super(imgType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, byte)
        super(imgType, self).insert(index, item)

setattr(RoboCompRGBD, "imgType", imgType)
class depthType(list):
    def __init__(self, iterable=list()):
        super(depthType, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(depthType, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(depthType, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(depthType, self).insert(index, item)

setattr(RoboCompRGBD, "depthType", depthType)
class DepthSeq(list):
    def __init__(self, iterable=list()):
        super(DepthSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, float)
        super(DepthSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, float)
        super(DepthSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, float)
        super(DepthSeq, self).insert(index, item)

setattr(RoboCompRGBD, "DepthSeq", DepthSeq)
class ColorSeq(list):
    def __init__(self, iterable=list()):
        super(ColorSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompRGBD.ColorRGB)
        super(ColorSeq, self).insert(index, item)

setattr(RoboCompRGBD, "ColorSeq", ColorSeq)
class PointSeq(list):
    def __init__(self, iterable=list()):
        super(PointSeq, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, RoboCompRGBD.PointXYZ)
        super(PointSeq, self).insert(index, item)

setattr(RoboCompRGBD, "PointSeq", PointSeq)

import handdetectionI
import apriltagsI



class Publishes:
    def __init__(self, ice_connector, topic_manager):
        self.ice_connector = ice_connector
        self.mprx={}
        self.topic_manager = topic_manager

        self.aprilbasedlocalization = self.create_topic("AprilBasedLocalization", RoboCompAprilBasedLocalization.AprilBasedLocalizationPrx)


    def create_topic(self, topic_name, ice_proxy):
        # Create a proxy to publish a AprilBasedLocalization topic
        topic = False
        try:
            topic = self.topic_manager.retrieve(topic_name)
        except:
            pass
        while not topic:
            try:
                topic = self.topic_manager.retrieve(topic_name)
            except IceStorm.NoSuchTopic:
                try:
                    topic = self.topic_manager.create(topic_name)
                except:
                    print(f'Another client created the {topic_name} topic? ...')
        pub = topic.getPublisher().ice_oneway()
        proxy = ice_proxy.uncheckedCast(pub)
        self.mprx[topic_name] = proxy
        return proxy

    def get_proxies_map(self):
        return self.mprx


class Requires:
    def __init__(self, ice_connector):
        self.ice_connector = ice_connector
        self.mprx={}

        self.CameraSimple = self.create_proxy("CameraSimpleProxy", RoboCompCameraSimple.CameraSimplePrx)

        self.RGBD = self.create_proxy("RGBDProxy", RoboCompRGBD.RGBDPrx)

    def get_proxies_map(self):
        return self.mprx

    def create_proxy(self, property_name, ice_proxy):
        # Remote object connection for
        try:
            proxy_string = self.ice_connector.getProperties().getProperty(property_name)
            try:
                base_prx = self.ice_connector.stringToProxy(proxy_string)
                proxy = ice_proxy.uncheckedCast(base_prx)
                self.mprx[property_name] = proxy
                return True, proxy
            except Ice.Exception:
                print('Cannot connect to the remote object (CameraSimple)', proxy_string)
                # traceback.print_exc()
                return False, None
        except Ice.Exception as e:
            console.print_exception(e)
            console.log(f'Cannot get {property_name} property.')
            return False, None


class Subscribes:
    def __init__(self, ice_connector, topic_manager, default_handler):
        self.ice_connector = ice_connector
        self.topic_manager = topic_manager

        self.AprilTags = self.create_adapter("AprilTagsTopic", apriltagsI.AprilTagsI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        adapter = self.ice_connector.createObjectAdapter(property_name)
        handler = interface_handler
        proxy = adapter.addWithUUID(handler).ice_oneway()
        topic_name = property_name.replace('Topic','')
        subscribe_done = False
        while not subscribe_done:
            try:
                topic = self.topic_manager.retrieve(topic_name)
                subscribe_done = True
            except Ice.Exception as e:
                console.log("Error. Topic does not exist (creating)", style="blue")
                time.sleep(1)
                try:
                    topic = self.topic_manager.create(topic_name)
                    subscribe_done = True
                except:
                    console.log(f"Error. Topic {Text(topic_name, style='red')} could not be created. Exiting")
                    status = 0
        qos = {}
        topic.subscribeAndGetPublisher(qos, proxy)
        adapter.activate()
        return adapter


class Implements:
    def __init__(self, ice_connector, default_handler):
        self.ice_connector = ice_connector
        self.handdetection = self.create_adapter("HandDetection", handdetectionI.HandDetectionI(default_handler))

    def create_adapter(self, property_name, interface_handler):
        adapter = self.ice_connector.createObjectAdapter(property_name)
        adapter.add(interface_handler, self.ice_connector.stringToIdentity(property_name.lower()))
        adapter.activate()


class InterfaceManager:
    def __init__(self, ice_config_file):
        # TODO: Make ice connector singleton
        self.ice_config_file = ice_config_file
        self.ice_connector = Ice.initialize(self.ice_config_file)
        self.topic_manager = self.init_topic_manager()
        self.status = 0
        self.parameters = {}
        for i in self.ice_connector.getProperties():
            self.parameters[str(i)] = str(self.ice_connector.getProperties().getProperty(i))
        self.requires = Requires(self.ice_connector)
        self.publishes = Publishes(self.ice_connector, self.topic_manager)
        self.implements = None
        self.subscribes = None



    def init_topic_manager(self):
        # Topic Manager
        proxy = self.ice_connector.getProperties().getProperty("TopicManager.Proxy")
        obj = self.ice_connector.stringToProxy(proxy)
        try:
            return IceStorm.TopicManagerPrx.checkedCast(obj)
        except Ice.ConnectionRefusedException as e:
            console.log(Text('Cannot connect to rcnode! This must be running to use pub/sub.', 'red'))
            exit(-1)

    def set_default_hanlder(self, handler):
        self.implements = Implements(self.ice_connector, handler)
        self.subscribes = Subscribes(self.ice_connector, self.topic_manager, handler)

    def get_proxies_map(self):
        result = {}
        result.update(self.requires.get_proxies_map())
        result.update(self.publishes.get_proxies_map())
        return result

    def destroy(self):
        if self.ice_connector:
            self.ice_connector.destroy()




