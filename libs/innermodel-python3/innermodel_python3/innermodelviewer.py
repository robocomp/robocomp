'''
For Rendering the innermodel scene
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
import time
import numpy as np
import pybullet as p
import pybullet_data
import cv2

from innermodel import InnerModel
from innermodelimu import InnerModelIMU
from innermodelrgbd import InnerModelRGBD
from innermodelnode import InnerModelNode
from innermodelmesh import InnerModelMesh
from innermodellaser import InnerModelLaser
from innermodelplane import InnerModelPlane
from innermodeljoint import InnerModelJoint
from innermodelvector import InnerModelVector
from innermodelcamera import InnerModelCamera
from innermodeltransform import InnerModelTransform
from innermodelomnirobot import InnerModelOmniRobot
from innermodeltouchsensor import InnerModelTouchSensor
from innermodelprismaticjoint import InnerModelPrismaticJoint
from innermodeldifferentialrobot import InnerModelDifferentialRobot

class BodyLink (object):
    '''Class containing all the info related to a link'''

    def __init__ (self, mass, collisionShapeIndex, visualShapeIndex, pos, orient, inertialFramePos,
                  inertialFrameOrient, parentIndex, jointType, jointAxis, texture, name):
        self.linkMass = mass
        self.linkCollisionShapeIndex = collisionShapeIndex
        self.linkVisualShapeIndex = visualShapeIndex
        self.linkPosition = pos
        self.linkOrientation = orient
        self.linkInertialFramePosition = inertialFramePos
        self.linkInertialFrameOrientation = inertialFrameOrient
        self.linkParentIndex = parentIndex
        self.linkJointType = jointType
        self.linkJointAxis = jointAxis
        self.linkTexture = texture
        self.linkName = name
        self.hasChildLink = False
        self.imChildNode = None
        self.imNode = None
        self.imParentNode = None

class BodyCamera (object):
    '''Class containing all the info related to a camera'''

    def __init__ (self, viewMatrix, projectionMatrix, eyePos, targetPos, upVector, imageSize):
        self.cameraViewMatrix = viewMatrix
        self.cameraProjectionMatrix = projectionMatrix
        self.cameraEyePosition = eyePos
        self.cameraTargetPosition = targetPos
        self.cameraUpVector = upVector
        self.cameraImageSize = imageSize
        self.imCameraNode = None

class BodyLaser (object):
    '''Class containing all the info about a laser'''

    def __init__ (self, pos, numRays, rayLength, rayBatchFrom, rayBatchTo):
        self.laserPosition = pos
        self.laserNumRays = numRays
        self.laserRayLength = rayLength
        self.laserRayBatchFrom = rayBatchFrom
        self.laserRayBatchTo = rayBatchTo
        self.imLaserNode = None

class BodyIMU (object):
    def __init__ (self, id, linkId, parentId, pos, orient):
        # orient in euler
        self.name = id
        self.linkId = linkId
        self.parentId = parentId
        self.pos = pos
        self.orient = orient
        self.prev_lin_vel = [0, 0, 0]
        self.prev_ang_vel = [0, 0, 0]
        self.last_t = 0
        self.imIMUNode = None

    def measure (self, id):
        curr_t = time.time()
        dt = curr_t - self.last_t
        self.last_t = curr_t
        lin_vel, ang_vel = p.getBaseVelocity (id)
        lin_accln = list ((np.array(lin_vel) - np.array(self.prev_lin_vel))/dt)
        ang_accln = list ((np.array(ang_vel) - np.array(self.prev_ang_vel))/dt)
        self.prev_lin_vel = lin_vel
        self.prev_ang_vel = ang_vel
        return (ang_vel, lin_accln, ang_accln)

class BodyJoint (object):
    def __init__ (self, id, tr, home, min_angle, max_angle, axis, parentLinkId):
        self.tr = tr
        self.id = id
        self.home = home
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.axis = axis
        self.parentLinkId = parentLinkId
        self.childLinkId = None
        self.imJointNode = None
        self.imParentNode = None
        self.imChildNode = None

    def update (self):
        pass

class MultiLinkBody (object):
    '''Class to represent a Multi Linked Body'''

    def __init__(self):
        '''Constructor having all the paramters for multibody in pybullet'''

        self.id = ''
        self.bodyId = None
        self.baseMass = 0
        self.baseCollisionShapeIndex = -1
        self.baseVisualShapeIndex = -1
        self.basePosition = [0,0,0]
        self.baseOrientation = p.getQuaternionFromEuler([0,0,0])
        self.imNode = None

        self.hasCamera = False
        self.hasLaser = False
        self.hasTouchSensor = False
        self.hasIMUs = False

        self.touchSensorLink = []
        self.links = []
        self.cameras = []
        self.lasers = []
        self.imus = []
        self.joints = []

    def updatePosAndOrient (self):
        self.basePosition = [self.imNode.tx/100.0, self.imNode.ty/100.0, self.imNode.tz/100.0]
        self.baseOrientation = p.getQuaternionFromEuler ([self.imNode.rx, self.imNode.ry,
                                                                                    self.imNode.rz])
        p.resetBasePositionAndOrientation (self.bodyId, self.basePosition, self.baseOrientation)

    def eulerFromNormal (self, normal):
        '''Returns rotation angles from normal vector of a plane'''
        normal = [normal[0], normal[1], normal[2]]
        if normal == [0, 0, 1]:
            euler = [0, 0, 0]
        elif normal == [0, 0, -1]:
            euler = [np.pi, 0, 0]
        elif normal == [1, 0, 0]:
            euler = [-np.pi/2, 0, -np.pi/2]
        elif normal == [-1, 0, 0]:
            euler = [-np.pi/2, 0, np.pi/2]
        elif normal == [0, 1, 0]:
            euler = [-np.pi/2, 0, 0]
        elif normal == [0, -1, 0]:
            euler = [np.pi/2, 0, 0]
        else:
            euler = [0, 0, 0]
        return euler

    def hexToInt (self, c):
        if (c == '1' or c == '2' or c == '3' or c == '4' or c == '5' \
            or c == '6' or c == '7' or c == '8' or c == '9' or c == '0'):
            return int (c)
        elif (c == 'A' or c == 'a'):
            return 10
        elif (c == 'B' or c == 'b'):
            return 11
        elif (c == 'C' or c == 'c'):
            return 12
        elif (c == 'D' or c == 'd'):
            return 13
        elif (c == 'E' or c == 'e'):
            return 14
        elif (c == 'F' or c == 'f'):
            return 15
        else:
            print (c)
            raise Exception ("invalid character")

    def readTexture (self, texture):
        '''Convert the texture into the color format taken by pybullet'''

        if (texture[0] == '#'):
            r = float (self.hexToInt(texture[1])*16 + self.hexToInt(texture[2]))/255.0
            g = float (self.hexToInt(texture[3])*16 + self.hexToInt(texture[4]))/255.0
            b = float (self.hexToInt(texture[5])*16 + self.hexToInt(texture[5]))/255.0
            color = [r, g, b, 1]
            return color
        else:
            textId = p.loadTexture (texture)
            return textId

    def includeLink (self, node, mass, visual, collision, position,
                     orientation, parentId, jointType, jointAxis, txt, name):
        '''Include a body with given parameters to the list'''

        link = BodyLink (mass, collision, visual, position, orientation, [0,0,0], [0,0,0,1],
                         parentId, jointType, jointAxis, txt, name)
        link.imNode = node
        if len (self.links) > 0 and self.links[len(self.links)-1].hasChildLink:
            link.imParentNode = self.links[len(self.links)-1].imNode
        self.links.append (link)

    def getJointAxis (self, axis):
        if (axis == 'z'):
            return [0, 0, 1]
        elif (axis == 'x'):
            return [1, 0, 0]
        elif (axis == 'y'):
            return [0, 1, 0]
        else:
            print ("invalid axis")
            return [0, 0, 0]

    def makeMesh (self, node, tr, parentId):
        linkMass = 1
        self.baseMass += linkMass
        meshScale = [node.scalex, node.scaley, node.scalez]
        visualShapeId = p.createVisualShape (shapeType=p.GEOM_MESH,
                                             fileName=node.meshPath,
                                             meshScale=meshScale)
        collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_MESH,
                                                   fileName=node.meshPath,
                                                   meshScale=meshScale)
        linkPosition = [node.tx/100.0 + tr.x(), node.ty/100.0 + tr.y(), node.tz/100.0 + tr.z()]
        linkOrientation = p.getQuaternionFromEuler ([node.rx + tr.rx(), node.ry + tr.ry(),
                                                        node.rz + tr.rz()])
        num = len (self.links)
        if num > 0 and self.links[num-1].hasChildLink:
            parentId = num
            linkJointType = p.JOINT_PLANAR
            self.joints[len(self.joints)-1].childLinkId = num+1
            self.joints[len(self.joints)-1].imChildNode = node
            self.joints[len(self.joints)-1].imParentNode = self.links[num-1].imNode
            self.links[num-1].imChildNode = node
            linkJointAxis = self.getJointAxis (self.joints[len(self.joints)-1].axis)
        else:
            linkJointType = p.JOINT_FIXED
            linkJointAxis = [0, 0, 1]

        self.includeLink (node, linkMass, visualShapeId, collisionShapeId, linkPosition,
                          linkOrientation, parentId, linkJointType, linkJointAxis, None, node.id)

    def makePlane (self, node, tr, parentId):
        linkMass = 1
        self.baseMass += linkMass
        size = [node.width/2,node.height/2,node.depth/2]
        texture = None
        color = None
        if (len(node.texture) != 0 and node.texture[0] == '#'):
            color = self.readTexture(node.texture)
        elif (len(node.texture) != 0):
            texture = self.readTexture(node.texture)

        visualShapeId = p.createVisualShape (shapeType=p.GEOM_BOX,
                                                rgbaColor=color,
                                                halfExtents=size)
        collisionShapeId = p.createCollisionShape (shapeType=p.GEOM_BOX,
                                                    halfExtents=size)
        linkPosition = [node.point[0]/100.0 + tr.x(), node.point[1]/100.0 + tr.y(),
                        node.point[2]/100.0 + tr.z()]
        euler = self.eulerFromNormal (node.normal)
        linkOrientation = p.getQuaternionFromEuler (euler)

        num = len (self.links)
        if num > 0 and self.links[num-1].hasChildLink:
            parentId = num
            linkJointType = p.JOINT_FIXED
            self.joints[len(self.joints)-1].childLinkId = num+1
            linkJointAxis = self.getJointAxis (self.joints[len(self.joints)-1].axis)
        else:
            linkJointType = p.JOINT_FIXED
            linkJointAxis = [0, 0, 1]

        self.includeLink (node, linkMass, visualShapeId, collisionShapeId, linkPosition,
                          linkOrientation, parentId, linkJointType, linkJointAxis, texture, node.id)

    def makeCamera (self, node, tr, parentId):
        self.hasCamera = True
        imageSize = [int(node.width), int(node.height)]
        viewMatrix = p.computeViewMatrix (cameraEyePosition=[tr.x(), tr.y(), tr.z()],
                                            cameraTargetPosition=[tr.x(), tr.y() + 10, tr.z()],
                                            cameraUpVector=[0, 0, 1])
        eyePos = [tr.x(), tr.y()-0.6, tr.z()]
        targetPos = [tr.x(), tr.y()-5, tr.z()]
        upVector = [0, 0, 1]
        projectionMatrix = p.computeProjectionMatrixFOV (fov=node.focal,
                                                            aspect=node.width/node.height,
                                                            nearVal=0.3,
                                                            farVal=5.1)
        camera = BodyCamera (viewMatrix, projectionMatrix, eyePos, targetPos, upVector, imageSize)
        camera.imCameraNode = node
        self.cameras.append (camera)

    def makeLaser (self, node, tr, parentId):
        self.hasLaser = True
        pos = [tr.x(), tr.y(), tr.z()]
        numRays = node.measures
        length = node.max/100
        rayFrom = []
        rayTo = []
        for i in range(node.measures):
            rayFrom.append([tr.x(), tr.y(), tr.z()])
            rayTo.append([
                tr.x() + length * math.sin(2. * math.pi * float(i) / node.measures),
                tr.y() + length * math.cos(2. * math.pi * float(i) / node.measures), tr.z()
            ])
        laser = BodyLaser (pos, numRays, length, rayFrom, rayTo)
        laser.imLaserNode = node
        self.lasers.append (laser)

    def makeIMU (self, node, tr, parentId):
        self.hasIMUs = True
        pos = [tr.x(), tr.y(), tr.z()]
        orient = [tr.rx(), tr.ry(), tr.rz()]
        imu = BodyIMU (node.id, None, parentId, pos, orient)
        imu.imIMUNode = node
        self.imus.append (imu)

    def makeJoint (self, node, tr, parentId):
        num = len(self.links)
        self.links[num-1].hasChildLink = True
        tr_ = [tr.x(), tr.y(), tr.z(), tr.rx(), tr.ry(), tr.rz()]
        joint = BodyJoint (node.id, tr_, node.home, node.min, node.max, node.axis, num)
        joint.imJointNode = node
        self.joints.append (joint)

        self.baseMass += node.mass

        for child in node.children:
            self.recursiveConstructor (child, tr, parentId)

    def recursiveConstructor (self, node, tr, parentId):
        '''Recursively construct all the parts in a body'''

        if isinstance (node, InnerModelMesh):
            self.makeMesh (node, tr, parentId)
        elif isinstance (node, InnerModelPlane):
            self.makePlane (node, tr, parentId)
        elif isinstance (node, (InnerModelRGBD, InnerModelCamera)):
            self.makeCamera (node, tr, parentId)
        elif isinstance (node, InnerModelLaser):
            self.makeLaser (node, tr, parentId)
        elif isinstance (node, InnerModelIMU):
            self.makeIMU (node, tr, parentId)
        elif isinstance (node, (InnerModelJoint, InnerModelPrismaticJoint)):
            self.makeJoint (node, tr, parentId)
        elif isinstance (node, InnerModelTouchSensor):
            self.hasTouchSensor = True
            parentId = node.parent.id
            self.touchSensorLink.append (parentId)
        else:
            self.baseMass += node.mass
            tr_ = InnerModelVector.vec6d (node.tx/100.0 + tr.x(), node.ty/100.0 + tr.y(),
                                          node.tz/100.0 + tr.z(), node.rx + tr.rx(),
                                          node.ry + tr.ry(), node.rz + tr.rz())
            for child in node.children:
                self.recursiveConstructor (child, tr_, parentId)

    def getClassType (self, node):
        '''Returns if an object is transform type or node type'''

        if isinstance (node, (InnerModelTransform, InnerModelDifferentialRobot, InnerModelJoint, \
                       InnerModelPrismaticJoint, InnerModelOmniRobot)):
            return 'Transform'
        else:
            return 'Node'

    def createBody (self, node):
        '''Create a separate body from the node'''

        assert (self.getClassType (node) == 'Transform')
        self.imNode = node
        self.id = node.id
        # self.baseMass = node.mass
        self.baseMass = 10
        self.basePosition = [node.tx/100.0, node.ty/100.0, node.tz/100.0]
        self.baseOrientation = p.getQuaternionFromEuler ([node.rx, node.ry, node.rz])
        tr = InnerModelVector.vec6d (0, 0, 0, 0, 0, 0)
        parentId = 0

        for child in node.children:
            self.recursiveConstructor (child, tr, parentId)

class InnerModelViewer (object):
    '''Class to view the rendered world from innermodel'''

    def __init__ (self, innermodel: 'InnerModel' = None):
        '''Constructor, takes innermodel'''

        self.innerModel = innermodel
        self.bodies = []
        self.worldNode = None
        if self.innerModel is not None:
            self.worldNode = self.innerModel.root
            self.construct (self.innerModel.root)

        self.initPyBullet()
        self.initialiseRender()

    def initPyBullet (self):
        p.connect(p.DIRECT) # GUI or DIRECT
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.loadURDF("plane.urdf")
        # p.setGravity(0, 0, 0)
        # p.setRealTimeSimulation(1)

    def MakeMultiLinkedBody (self, node):
        '''Method to make a multilinked body'''

        body = MultiLinkBody()
        body.createBody (node)
        return body

    def MakeWorld (self):
        '''To construct the world with all the bodies in it'''

        if self.worldNode is None:
            self.construct (self.innerModel.root)
        root = self.worldNode

        for child in root.children:
            body = self.MakeMultiLinkedBody (child)
            self.bodies.append (body)

    def construct (self, node):
        '''Find the node representing the world'''

        for child in self.innerModel.root.children:
            if (child.id == 'world'):
                self.worldNode = child
                break

    # github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py
    def getRayFromTo(self, mouseX, mouseY):
        '''Given a point on the GUI, get a ray from the current position to infinity and beyond'''

        width, height, _, _, _, camForward, horizon, \
                                    vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
        camPos = [
            camTarget[0] - dist * camForward[0],
            camTarget[1] - dist * camForward[1],
            camTarget[2] - dist * camForward[2]
        ]
        farPlane = 10000
        rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]),
                      (camTarget[2] - camPos[2])]
        lenFwd = math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] +
                            rayForward[2] * rayForward[2])
        invLen = farPlane * 1. / lenFwd
        rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
        rayFrom = camPos
        oneOverWidth = float(1) / float(width)
        oneOverHeight = float(1) / float(height)

        dHor = [horizon[0]*oneOverWidth, horizon[1]*oneOverWidth, horizon[2]*oneOverWidth]
        dVer = [vertical[0]*oneOverHeight, vertical[1]*oneOverHeight, vertical[2]*oneOverHeight]
        _ = [
            rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
        ]
        ortho = [
        -0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
        -0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
        -0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
        ]

        rayTo = [
            rayFrom[0] + rayForward[0] + ortho[0], rayFrom[1] + rayForward[1] + ortho[1],
            rayFrom[2] + rayForward[2] + ortho[2]
        ]
        lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
        alpha = math.atan(lenOrtho / farPlane)
        return rayFrom, rayTo, alpha

    # github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py
    def getPointCloud (self, near, far, width, height, stepX, stepY, depthImg):
        '''Method to get point cloud from depth image'''

        imgW = depthImg.shape[1]
        imgH = depthImg.shape[0]
        pointcloud = []

        for w in range(0, imgW, stepX):
            for h in range(0, imgH, stepY):
                rayFrom, rayTo, alpha = self.getRayFromTo(w * (width / imgW), h * (height / imgH))
                rf = np.array(rayFrom)
                rt = np.array(rayTo)
                vec = rt - rf
                l = np.sqrt(np.dot(vec, vec))
                depthImg = float(depthImg[h, w])
                depth = far * near / (far - (far - near) * depthImg)
                depth /= math.cos(alpha)
                newTo = (depth / l) * vec + rf
                pointcloud.append ([rayFrom, newTo])

        return pointcloud

    def readIMU (self, body):
        data = []
        if body.hasIMU:
            for imu in body.imus:
                data.append (imu.measure(body.bodyId))
        return data

    def renderImage (self):
        '''Render the image for bodies having camera'''

        for body in self.bodies:
            if body.hasCamera:
                for i in range (len(body.cameras)):
                    (base_pos, orien) = p.getBasePositionAndOrientation (body.bodyId)
                    mat = p.getMatrixFromQuaternion (orien)
                    mat = np.transpose (np.array (mat).reshape ((3,3)))
                    cam_pos = mat.dot(np.array(body.cameras[i].cameraEyePosition).reshape((3,)))
                    cameraEyePosition = [base_pos[0] + cam_pos[0],
                                         base_pos[1] + cam_pos[1],
                                         base_pos[2] + cam_pos[2]]
                    cam_target_pos = \
                            mat.dot(np.array(body.cameras[i].cameraTargetPosition).reshape((3,)))
                    cameraTargetPosition = [cam_target_pos[0] + base_pos[0],
                                            cam_target_pos[1] + base_pos[1],
                                            cam_target_pos[2] + base_pos[2]]
                    cameraUpVector = mat.dot(np.array(body.cameras[i].cameraUpVector).reshape((3,)))
                    viewMatrix = p.computeViewMatrix (cameraEyePosition=cameraEyePosition,
                                                      cameraTargetPosition=cameraTargetPosition,
                                                      cameraUpVector=cameraUpVector)
                    images = p.getCameraImage (width=body.cameras[i].cameraImageSize[0],
                                          height=body.cameras[i].cameraImageSize[1],
                                          viewMatrix=viewMatrix,
                                          projectionMatrix=body.cameras[i].cameraProjectionMatrix)

                    rgb_img = cv2.cvtColor (images[2], cv2.COLOR_BGR2RGB)

                    cv2.imshow (body.id+" camera no.: "+str(i), rgb_img)
                    k = cv2.waitKey (1)
                    if k == 27:
                        cv2.destroyAllWindows ()


    def detectLaserCollisions (self):
        '''Detect collision of laser rays and render it on GUI'''

        for body in self.bodies:
            if body.hasLaser:
                for i in range (len(body.lasers)):
                    (pos, _) = p.getBasePositionAndOrientation (body.bodyId)
                    rayFrom = body.lasers[i].laserRayBatchFrom
                    rayTo = body.lasers[i].laserRayBatchTo

                    newFrom = []
                    newTo = []

                    for k in range (len(rayFrom)):
                        newFrom.append ([rayFrom[k][0] + pos[0], rayFrom[k][1] + pos[1],
                                         rayFrom[k][2] + pos[2]])
                        newTo.append ([rayTo[k][0] + pos[0], rayTo[k][1] + pos[1],
                                       rayTo[k][2] + pos[2]])

                    results = p.rayTestBatch(newFrom, newTo)

                    for j in range(len(rayFrom)):
                        hitObjectUid = results[j][0]

                        if (hitObjectUid < 0):
                            p.addUserDebugLine(newFrom[j], newTo[j], lineColorRGB=[1, 0, 0],
                                               lifeTime=0.1)
                        else:
                            hitPosition = results[j][3]
                            p.addUserDebugLine(newFrom[j], hitPosition, lineColorRGB=[0, 1, 0],
                                               lifeTime=0.1)

    def initialiseRender(self):
        '''Render the scene'''

        self.MakeWorld()

        for body in self.bodies:
            linkMasses = [link.linkMass for link in body.links]
            linkCollisionShapeIndices = [link.linkCollisionShapeIndex for link in body.links]
            linkVisualShapeIndices = [link.linkVisualShapeIndex for link in body.links]
            linkPositions = [link.linkPosition for link in body.links]
            linkOrientations = [link.linkOrientation for link in body.links]
            linkInertialFramePositions = [link.linkInertialFramePosition for link in body.links]
            linkInertialFrameOrientations = \
                                        [link.linkInertialFrameOrientation for link in body.links]
            linkParentIndices = [link.linkParentIndex for link in body.links]
            linkJointTypes = [link.linkJointType for link in body.links]
            linkJointAxis = [link.linkJointType for link in body.links]

            bodyId = p.createMultiBody(baseMass=body.baseMass,
                            baseCollisionShapeIndex=body.baseCollisionShapeIndex,
                            baseVisualShapeIndex=body.baseVisualShapeIndex,
                            basePosition=body.basePosition,
                            baseOrientation=body.baseOrientation,
                            linkMasses=linkMasses,
                            linkCollisionShapeIndices=linkCollisionShapeIndices,
                            linkVisualShapeIndices=linkVisualShapeIndices,
                            linkPositions=linkPositions,
                            linkOrientations=linkOrientations,
                            linkInertialFramePositions=linkInertialFramePositions,
                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                            linkParentIndices=linkParentIndices,
                            linkJointTypes=linkJointTypes,
                            linkJointAxis=linkJointAxis)
            body.bodyId = bodyId

            for i in range (len(body.links)):
                if (body.links[i].linkTexture is not None):
                    textId = body.links[i].linkTexture
                    p.changeVisualShape (bodyId, i, textureUniqueId=textId)

    def resetPosAndOrient (self):
        for body in self.bodies:
            body.updatePosAndOrient ()

    def renderDebugWindow (self):
        W = 1024
        H = 768
        VM = (0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574, \
              0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004, \
              0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0)
        PM = (0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, \
              0.0, 0.0, -0.02000020071864128, 0.0)

        debug_images = p.getCameraImage (W, H, VM, PM)
        debug_rgb = cv2.cvtColor (debug_images[2], cv2.COLOR_BGR2RGB)

        cv2.imshow ("debug_image", debug_rgb)
        k = cv2.waitKey (1)
        if k == 27:
            cv2.destroyAllWindows ()

    def render(self):
        p.stepSimulation ()
        self.resetPosAndOrient ()
        self.renderDebugWindow ()
        self.renderImage ()
        self.detectLaserCollisions ()
