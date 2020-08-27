'''
Class representing a complete innermodel scene
Author: ksakash@github.com (Akash Kumar Singh)
'''

import os
import copy
import numpy as np

from innermodelnode import InnerModelNode
from innermodelmatrix import InnerModelMatrix
from innermodeltransform import InnerModelTransform
from innermodelmesh import InnerModelMesh
from innermodelrtmatrix import InnerModelRTMat
from innermodelvector import InnerModelVector
from innermodeltouchsensor import InnerModelTouchSensor
from innermodeljoint import InnerModelJoint
from innermodelprismaticjoint import InnerModelPrismaticJoint
from innermodeldifferentialrobot import InnerModelDifferentialRobot
from innermodelomnirobot import InnerModelOmniRobot
from innermodelcamera import InnerModelCamera
from innermodelrgbd import InnerModelRGBD
from innermodelimu import InnerModelIMU
from innermodellaser import InnerModelLaser
from innermodelplane import InnerModelPlane
from innermodeldisplay import InnerModelDisplay
from innermodelmesh import InnerModelMesh
from innermodelpointcloud import InnerModelPointCloud
from innermodelreader import InnerModelReader
from innermodelrotmatrix import Rot3D

class InnerModel(object):
    '''Class representing a scene'''

    def __init__ (self, xmlFilePath: str = None):
        '''
        :param xmlFilePath: path of the xml file representing innermodel
        '''

        self.path = xmlFilePath
        self.hash = dict()

        self.root = InnerModelTransform ('root', 'static', 0, 0, 0, 0, 0, 0)
        self.root.parent = None
        self.setRoot (self.root)
        self.root.innerModel = self
        self.hash['root'] = self.root

        self.localHashTr = dict()
        self.localHashRot = dict()
        self.listA = []
        self.listB = []

        if self.path is not None:
            self.root = None
            InnerModelReader.load (file=self.path, model=self)

    def open (self, xmlFilePath: str) -> bool:
        '''Read the xml file'''

        return InnerModelReader.load (file=xmlFilePath, model=self)

    def save (self, path: str) -> bool:
        '''Save the info into a file'''

        f = open (path, "w+")
        self.root.save (f, 0)
        f.close ()

        return True

    def copy (self) -> 'InnerModel':
        '''Returns a copy of the object'''

        ret = copy.deepcopy (self)
        return ret

    def changeHash (self, new_id: str, node: 'InnerModelNode'):
        '''Map another node corresponding to the given new_id'''

        self.hash[new_id] = node

    def setRoot (self, node: 'InnerModelNode'):
        '''Set the root of the model'''

        self.root = node
        self.hash['root'] = self.root
        self.root.parent = None

    def update (self): # redundant
        '''update parameters of all the nodes in the innermodel'''

        self.root.update()
        self.cleanUpTables()

    def updateRotation (self, id: str, rx: float, ry: float, rz: float):
        '''Update rotation paramters of the node with the given id'''

        if self.hash[id] is None:
            print ("There is no such %s node" % id)
        else:
            aux = self.hash[id]
            aux.updateRotation (rx, ry, ry)

    def updateTranslation (self, id: str, tx: float, ty: float, tz: float):
        '''Update translation parameters of the node with the given id'''

        if self.hash[id] is None:
            print ("There is no such %s node" % id)
        else:
            aux = self.hash[id]
            aux.updateTranslation (tx, ty, ty)

    def updatePlane (self, id: str, nx: float, ny: float, nz: float,
                                         px: float, py: float, pz: float):
        '''Update paramters of the plane with given id'''

        if self.hash[id] is None:
            print ("There is no such %s node" % id)
        else:
            aux = self.hash[id]
            aux.update (nx, ny, nz, px, py, pz)

    def updateTransform (self, id: str, tx: float, ty: float, tz: float,
                                                 rx: float, ry: float, rz: float):
        '''Update transformation parameters'''

        if self.hash[id] is None:
            print ("There is no such %s node" % id)
        else:
            aux = self.hash[id]
            aux.update (tx, ty, tz, rx, ry, ry)

    def cleanUpTables (self): # redundant
        '''Clear the hash tables'''

        self.localHashRot.clear()
        self.localHashTr.clear()

    def updateJointValue (self, jointId: str, angle: float, force: bool = False):
        '''Update the joint node parameters'''

        self.cleanUpTables()
        joint = self.hash[jointId]

        if joint is not None:
            joint.setAngle (angle, force)
        else:
            print ("There is no such %s node" % jointId)

    def updatePrismaticJointPosition (self, jointId: str, position: float):
        '''Update the prismatric joint position'''

        self.cleanUpTables()
        pj = self.hash[jointId]

        if pj is not None:
            pj.setPosition (position)
        else:
            print ("There is no such %s node" % jointId)

    def newTransform (self, id: str, engine: str, parent: 'InnerModelNode',
                    mass: float, tx: float, ty: float, tz: float, rx: float,
                    ry: float, rz: float) -> 'InnerModelTransform':
        '''Returns a new transform node with the given parameters'''

        if (id in self.hash):
            raise Exception ("InnerModel.newTransform: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)

        newnode = InnerModelTransform (id=id, engine=engine, tx=tx, ty=ty, tz=tz,
                                        rx=rx, ry=ry, rz=rz, mass=mass, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newTouchSensor (self, id: str, parent: 'InnerModelTransform', stype: str,
                        nx: float, ny: float, nz: float, min: float, max: float,
                        port: int) -> 'InnerModelTouchSensor':
        '''Returns a touch sensor with the given arguments'''

        if (id in self.hash):
            raise Exception ("InnerModel.newTouchSensor: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelTouchSensor (id=id, stype=stype, nx=nx, ny=ny, nz=nz, min=min, max=max,
                                        port=port, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newJoint (self, id: str, parent: 'InnerModelTransform', lx: float, ly: float,
                  lz: float, hx:float, hy: float, hz: float, tx: float, ty: float,
                  tz: float, rx: float, ry: float, rz: float, min: float, max: float,
                  port: int, axis: str, home: float) -> 'InnerModelJoint':
        '''Returns a joint object with the given paramter'''

        if (id in self.hash):
            raise Exception ("InnerModel.newJoint: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelJoint (id=id, lx=lx, ly=ly, lz=lx, hx=hx, hy=hy, hz=hz, tx=tx, ty=ty,
                                   tz=tz, rx=rx, ry=ry, rz=rz, min=min, max=max, port=port, axis=axis,
                                   home=home, parent=parent)
        self.hash[id] = newnode
        return newnode

    def newPrismaticJoint (self, id: str, min: float, max: float, value: float,
                           offset: float, port: int, axis: str, home: float,
                           parent: 'InnerModelTransform') -> 'InnerModelPrismaticJoint':
        '''Returns a new prismatic joint object with the given parameters'''

        if (id in self.hash):
            raise Exception ("InnerModel.newPrismaticJoint: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelPrismaticJoint (id, min, max, value, offset, port, axis, home, parent)
        self.hash[id] = newnode
        return newnode

    def newDifferentialRobot (self, id: str, tx: float, ty: float, tz: float, rx: float,
                              ry: float, rz: float, port: int, noise: float, collide: bool,
                              parent: 'InnerModelNode') -> 'InnerModelDifferentialRobot':
        '''Returns a new differential robot object with the given parameters'''

        if (id in self.hash):
            raise Exception ("InnerModel.newDifferentialRobot: Error: Trying to insert a node with \
                              an already-existing key: %s"%id)
        newnode = InnerModelDifferentialRobot (id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent)
        self.hash[id] = newnode
        return newnode

    def newOmniRobot (self, id: str, tx: float, ty: float, tz: float,
                      rx: float, ry: float, rz: float, port: int, noise: float,
                      collide: bool, parent) -> 'InnerModelOmniRobot':
        '''Returns a new omni robot with the given parameters'''

        if (id in self.hash):
            raise Exception ("InnerModel.newOmniRobot: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelOmniRobot (id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent)
        self.hash[id] = newnode
        return newnode

    def newCamera (self, id: str, width: float, height: float, focal: float) -> 'InnerModelCamera':
        '''Returns a new camera object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newCamera: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelCamera (id, width, height, focal, self, parent=None)
        self.hash[id] = newnode
        return newnode

    def newRGBD (self, id: str, width: float, height: float, focal: float, noise: float, port: int,
                 ifconfig: str, parent: 'InnerModelNode') -> 'InnerModelRGBD':
        '''Returns a RGBD object'''

        if (noise < 0):
            raise Exception ("InnerModel.newRGBD: noise can't have negative values")
        if (id in self.hash):
            raise Exception ("InnerModel.newRGBD: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelRGBD (id, width, height, focal, noise, port, ifconfig, self, parent)
        self.hash[id] = newnode
        return newnode

    def newIMU (self, id: str, port: int, parent: 'InnerModelNode') -> 'InnerModelIMU':
        '''Returns a new IMU object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newIMU: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelIMU (id, port, parent)
        self.hash[id] = newnode
        return newnode

    def newLaser (self, id: str, port: int, min: float, max: float, angle: float,
                  measures: float, ifconfig: str, parent: 'InnerModelNode') -> 'InnerModelLaser':
        '''Returns a new laser object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newLaser: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelLaser (id, port, min, max, angle, measures, ifconfig, self, parent)
        self.hash[id] = newnode
        return newnode

    def newMesh (self, id: str, parent: 'InnerModelNode', path: str,
                scalex: float, scaley: float, scalez: float, render: int,
                tx: float, ty: float, tz: float, rx: float, ry: float,
                rz: float, collidable: bool) -> 'InnerModelMesh':
        '''Returns a new mesh object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newMesh: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelMesh (id, path, scalex, scaley, scalez, render, tx, ty, tz, rx, ry, rz,
                                  collidable, parent)
        self.hash[id] = newnode
        return newnode

    def newPointCloud (self, id: str, parent: 'InnerModelNode') -> 'InnerModelPointCloud':
        '''Returns a new point cloud object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newPointCloud: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelPointCloud (id, parent)
        self.hash[id] = newnode
        return newnode

    def newPlane (self, id: str, texture: str, width: float, height: float, depth: float,
                  repeat: int, nx: float, ny: float, nz: float, px: float, py: float, pz: float,
                  collidable: bool, parent: 'InnerModelNode') -> 'InnerModelPlane':
        '''Returns a new plane object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newPlane: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelPlane (id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz,
                                   collidable, parent)
        self.hash[id] = newnode
        return newnode

    def newDisplay (self, id: str, port: int, texture: str, width: float,
                    height: float, depth: float, repeat: int, nx: float,
                    ny: float, nz: float, px: float, py: float, pz: float,
                    collidable: bool, parent: 'InnerModelNode') -> 'InnerModelDisplay':
        '''Returns a new display object'''

        if (id in self.hash):
            raise Exception ("InnerModel.newDisplay: Error: Trying to insert a node with an \
                              already-existing key: %s"%id)
        newnode = InnerModelDisplay (id, port, texture, width, height, depth, repeat, nx, ny, nz,
                                    px, py, pz, collidable, parent)
        self.hash[id] = newnode
        return newnode

    def getTransform (self, id: str) -> 'InnerModelTransform':
        '''Returns transform corresponding to the given id'''

        return self.getNode (id)

    def getMesh (self, id: str) -> 'InnerModelMesh':
        '''Returns mesh corresponding to the given id'''

        return self.getNode (id)

    def getJoint (self, id: str) -> 'InnerModelJoint':
        '''Returns joint corresponding to the given id'''

        return self.getNode (id)

    def getPrismaticJoint (self, id: str) -> 'InnerModelPrismaticJoint':
        '''Returns prismatic joint corresponding to the given id'''

        return self.getNode (id)

    def getTouchSensor (self, id: str) -> 'InnerModelTouchSensor':
        '''Returns touch sensor corresponding to the given id'''

        return self.getNode (id)

    def getDifferentialRobot (self, id: str) -> 'InnerModelDifferentialRobot':
        '''Returns differential robot corresponding to the given id'''

        return self.getNode (id)

    def getOmniRobot (self, id: str) -> 'InnerModelOmniRobot':
        '''Returns omni robot corresponding to the given id'''

        return self.getNode (id)

    def getCamera (self, id: str) -> 'InnerModelCamera':
        '''Returns camera corresponding to the given id'''

        return self.getNode (id)

    def getRGBD (self, id: str) -> 'InnerModelRGBD':
        '''Returns RGBD object corresponding to the given id'''

        return self.getNode (id)

    def getIMU (self, id: str) -> 'InnerModelIMU':
        '''Returns IMU corresponding to the given id'''

        return self.getNode (id)

    def getLaser (self, id: str) -> 'InnerModelLaser':
        '''Returns laser object corresponding to the given id'''

        return self.getNode (id)

    def getPlane (self, id: str) -> 'InnerModelPlane':
        '''Returns plane corresponding to the given id'''

        return self.getNode (id)

    def getPointCloud (self, id: str) -> 'InnerModelPointCloud':
        '''Returns point cloud corresponding to the given id'''

        return self.getNode (id)

    def transform (self, destId: str, origId: str,
                   origVec: 'InnerModelVector' = None) -> 'InnerModelVector':
        '''Get the transformation between two nodes with given ids'''

        if origVec is None:
            origVec = InnerModelVector.vec3d (0, 0, 0)

        if origVec.size() == 3:
            m = self.getTransformationMatrix(destId, origId)
            v = origVec.toHomogeneousCoordinates()
            vt_ = m.dot (v)
            vt = InnerModelVector ((4,))
            np.copyto (vt, vt_)
            return vt.fromHomogeneousCoordinates()
        else:
            M = self.getTransformationMatrix (destId, origId)
            a = M.dot (origVec.subVector(0, 2).toHomogeneousCoordinates()).fromHomogeneousCoordinates()
            R = Rot3D (origVec[3], origVec[4], origVec[5])

            b = (M.getSubmatrix(0,2,0,2).dot(R)).extractAnglesR_min()

            ret = InnerModelVector.vec6d (a[0], a[1], a[2], b[3], b[4], b[5])
            return ret

    def transform6D (self, destId: str, orgId: str,
                    origVec: 'InnerModelVector' = None) -> 'InnerModelVector':
        '''Get the transformation between two nodes with given ids'''

        if origVec is not None:
            assert (origVec.size() == 6)
            return self.transform (destId, orgId, origVec)
        else:
            v = InnerModelVector.vec6d (0, 0, 0, 0, 0, 0)
            return self.transform (destId, orgId, v)

    def setLists (self, destId: str, origId: str):
        a = self.hash [origId]
        b = self.hash [destId]

        if (a is None):
            raise Exception ("cannot find node: %s"%origId)
        if (b is None):
            raise Exception ("cannot find node: %s"%destId)

        minLevel = min (a.level, b.level)
        self.listA.clear()

        while (a.level >= minLevel):
            self.listA.insert (0, a)
            if a.parent is None:
                break
            a = a.parent

        self.listB.clear()

        while (b.level >= minLevel):
            self.listB.insert (0, b)
            if b.parent is None:
                break
            b = b.parent

        while (b != a):
            self.listA.append (a)
            self.listB.insert (0, b)
            a = a.parent
            b = b.parent

    def getTransformationMatrix (self, to: str, fr: str) -> 'InnerModelRTMat':
        '''Get transformation matrix betwn two nodes with given ids'''

        ret = InnerModelRTMat.getInnerModelRTMat (0, 0, 0, 0, 0, 0)
        self.setLists (fr, to)
        for node in self.listA:
            ret = node.rtmat.dot (ret)
        for node in self.listB:
            ret = node.rtmat.invert().dot (ret)
        return ret

    def getRotationMatrixTo (self, to: str, fr: str) -> 'Rot3D':
        '''Get rotation matrix betwn two nodes with given ids'''

        rret = InnerModelMatrix.identity (3)

        self.setLists (fr, to)
        for node in self.listA:
            rret = node.rmat.getR().dot (rret)
        for node in self.listB:
            rret = node.rmat.getR().getTranspose().dot (rret)

        return rret

    def getTranslationVectorTo (self, to: str, fr: str) -> 'InnerModelVector':
        '''Get translation vector betwn two nodes with the given ids'''

        m = self.getTransformationMatrix (to, fr)
        return m.getCol (3)

    def rotationAngles (self, destId: str, origId: str) -> 'InnerModelVector':
        '''Get rotation angles betwn two nodes with the given ids'''

        return self.getTransformationMatrix(destId, origId).extractAnglesR()

    def getIDKeys (self):
        '''Get keys of the hash map'''

        return self.hash.keys()

    def getNode (self, id: str):
        '''Get node with the given id'''

        if id in self.hash:
            return self.hash[id]
        else:
            return None

    def removeSubTree (self, node: 'InnerModeNode', l: list):
        '''Remove a subtree'''

        for child in node.children:
            self.removeSubTree (child, l)
        node.parent.children.removeOne (node)
        l.append (node.id)
        self.removeNode (node.id)

    def removeNode (self, id):
        '''Remove node with the given id'''

        self.hash.pop(id)

    def moveSubTree (self, nodeSrc: 'InnerModelNode', nodeDst: 'InnerModeNode'):
        nodeSrc.parent.children.removeOne (nodeSrc)
        nodeDst.addChild (nodeSrc)
        nodeSrc.setParent (nodeDst)
        self.computeLevels (nodeDst)

    def getSubTree (self, node: 'InnerModelNode', l: list):
        for child in node.children:
            l.append (child)
            self.getSubTree (child, l)

    def computeLevels (self, node: 'InnerModelNode'):
        if node.parent is not None:
            node.level = node.parent.level+1
        for child in node.children:
            self.computeLevels (child)

    def getRoot (self):
        '''Returns a pointer to the root'''

        return self.root

    def getParentIdentifier (self, id: str) -> str:
        '''Returns id of the parent of node with given id'''

        n = self.getNode (id)
        if n is not None:
            if n.parent is not None:
                return n.parent.id
        return ""

    def printTree (self, s: str = '', verbose: bool = False):
        '''Print the whole innermodel tree'''

        self.root.printTree (s, verbose)
