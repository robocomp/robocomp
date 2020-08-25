from innermodel import InnerModel
from innermodelviewer import InnerModelViewer
from innermodeltransform import InnerModelTransform
from innermodeljoint import InnerModelJoint
from innermodelplane import InnerModelPlane
from innermodelmesh import InnerModelMesh
from innermodelcamera import InnerModelCamera
from innermodelpointcloud import InnerModelPointCloud

class InnerModelDraw (object):
    def __init__ (self):
        pass

    @staticmethod
    def addMesh_ignoreExisting (innerviewer: 'InnerModelViewer', a: str, parent: str,
                                t: 'InnerModelVector', r: 'InnerModelVector', path: str,
                                scale: 'InnerModelVector'):
        pass

    @staticmethod
    def addTransform (innerviewer: 'InnerModelViewer', a: str, b: str):
        pass

    @staticmethod
    def addTransform_ignoreExisting (innerviewer: 'InnerModelViewer', a: str, b: str):
        pass

    @staticmethod
    def drawLine (innerviewer: 'InnerModelViewer', name: str, parent: str,
                  normalVector: 'InnerModelVector', center: 'InnerModelVector',
                  length: float, width: float, texture: str = '#550000'):
        pass

    @staticmethod
    def drawLine2Points (innerviewer: 'InnerModelViewer', name: str, parent: str,
                         p1: 'InnerModelVector', p2: 'InnerModelVector', width: float,
                         texture: str):
        pass

    @staticmethod
    def removeObject (innerviewer: 'InnerModelViewer', name: str):
        pass

    @staticmethod
    def removeNode (innerviewer: 'InnerModelViewer', item: str):
        pass

    @staticmethod
    def addPlane_ignoreExisting (innerviewer: 'InnerModelViewer', a: str, b: str,
                                 p: 'InnerModelVector', n: 'InnerModelVector', texture: str,
                                 size: 'InnerModelVector'):
        pass

    @staticmethod
    def addPlane_notExisting (innerviewer: 'InnerModelViewer', a: str, b: str,
                              p: 'InnerModelVector', n: 'InnerModelVector', texture: str,
                              size: 'InnerModelVector'):
        pass

    @staticmethod
    def setScale (innerviewer: 'InnerModelViewer', item: str, scaleX: float, scaleY: float,
                  scaleZ: float):
        pass

    @staticmethod
    def setPlaneTexture (innerviewer: 'InnerModelViewer', item: str, texture: str):
        pass

    @staticmethod
    def addJoint (innerviewer: 'InnerModelViewer', item: str, base: str, t: 'InnerModelVector',
                  r: 'InnerModelVector', axis: str):
        pass
