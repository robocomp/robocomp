import math
import numpy as np
import copy

from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector
from innermodelcam import InnerModelCam
from innermodelmatrix import InnerModelMatrix

class InnerModel (object):
    pass

class InnerModelCamera (InnerModelNode):
    def __init__ (self, id: str, width: float, height: float, focal: float,
                  innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelCamera, self).__init__(id, parent)
        self.innerModel = innermodel
        self.camera = InnerModelCam (Fx=focal, Fy=focal, Ox=width/2, Oy=height/2)
        self.camera.setSize (width, height)
        self.width = width
        self.height = height
        self.focal = focal

    def __repr__ (self):
        s = "InnermodelCamera, id: {}, width: {}, height: {}, focal: {}".format(self.id, self.width,
                                                                            self.height, self.focal)
        return s

    def print (self, verbose): # redundant
        if verbose:
            print (self)

    def save (self, out, tabs):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<camera id=\"" + self.id + "\" width=\"" + "%.9f" % self.width + "\" height=\"" + \
             "%.9f" % self.height + "\" focal=\"" + "%.9f" % self.focal + "\" />\n"

        out.write (s)

    def getWidth (self) -> float:
        return self.width

    def getHeight (self) -> float:
        return self.height

    def getFocal (self) -> float:
        return self.focal

    def getSize (self) -> float:
        return self.width*self.height

    def updateValues (self, width: float, height: float, focal: float):
        self.width = width
        self.height = height
        self.focal = focal
        self.camera = InnerModelCam (Fx=focal, Fy=focal, Ox=width/2, Oy=height/2)
        self.camera.setSize (width, height)

    def project (self, origVec: 'InnerModelVector', reference: str = None) -> 'InnerModelVetor':
        if reference is not None:
            origVec = self.innerModel.transform (self.id, reference, origVec)
        pc = self.camera.project (origVec)
        v = InnerModelVector.vec3d (pc[0], pc[1], origVec.norm2())
        return v

    def backProject (self, cameraId: str, coord: 'InnerModelVector') -> 'InnerModelVector':
        return self.camera.getRayHomogeneous(coord)

    def imageCoordToAngles (self, cameraId: str, coord: 'InnerModelVector',
                            anglesRefs: float) -> (float, float):
        ray = self.backProject (cameraId, coord)
        finalRay = self.innerModel.getRotationMatrixTo (anglesRefs, cameraId).dot (ray)
        pan = math.atan2 (finalRay[0], finalRay[2])
        tilt = math.atan2 (finalRay[1], finalRay[2])
        return (pan, tilt)

    def anglesToImageCoord (self, cameraId: str, pan: float, tilt: float,
                            anglesRefs: float) -> 'InnerModelVector':
        p = InnerModelVector ((3,))
        ray = InnerModelVector ((3,))

        p[0] = math.tan (pan)
        p[1] = math.tan (tilt)
        p[2] = 1

        ray = self.innerModel.getRotationMatrixTo (cameraId, anglesRefs).dot (p)
        ray[0] = ray[0]/ray[2]
        ray[1] = ray[1]/ray[2]
        ray[2] = 1

        return self.project (cameraId, ray)

    def imageCoordPlusDepthTo (self, cameraId: str, coord: 'InnerModelVector',
                               depth: float, to: str) -> 'InnerModelVector':
        p = self.backProject (cameraId, coord) * depth
        if (p.size() > 0):
            return self.innerModel.transform (to, cameraId, p)
        return p

    def projectFromCameraToPlane (self, to: str, coord: 'InnerModelVector', cameraId: str,
                                  vPlane: 'InnerModelVector', dist: float) -> 'InnerModelVector':
        pCam = InnerModelVector ((3,))
        res = InnerModelVector ((3,))

        pCam[0] = -coord[0] + self.getWidth()/2
        pCam[1] = -1*(coord[1] - self.getHeight()/2)
        pCam[2] = self.getFocal()
        pDest = self.innerModel.transform (to, cameraId, pCam)
        pCent = self.innerModel.transform (to, cameraId, InnerModelVector.vec3d(0,0,0))
        direc = pDest - pCent
        dxz = direc[0]/direc[2]
        dyz = direc[1]/direc[2]

        res[2] = dist + vPlane[0]*(dxz*pCent[2] - pCent[0]) + vPlane[1]*(dyz*pCent[2] - pCent[1])
        res[2] = res[2]/(vPlane[0]*dxz + vPlane[1]*dyz + vPlane[2])
        res[0] = dxz*(res[2] - pCent[2]) + pCent[0]
        res[1] = dyz*(res[2] - pCent[2]) + pCent[1]
        return res

    def horizonLine (self, planeId: str, cameraId: str, heightOffset: float = 0) -> 'InnerModelVector':
        plane = self.innerModel.getNode (planeId)
        camera = self.innerModel.getNode (cameraId)

        rtm = self.innerModel.getRotationMatrixTo (cameraId, planeId)
        vec = InnerModelVector.vec3d (plane.normal[0], plane.normal[1], plane.normal[2])
        normal = rtm.dot (vec)

        if normal[1] <= 0.0000002:
            raise Exception ("InnerModelCamera: something wrong")

        p1 = InnerModelVector.vec3d (0, 0, 0)
        p2 = InnerModelVector.vec3d (0, 0, 0)

        p1[2] = p2[2] = 1000
        if normal[1] > 0.0000001:
            p1[1] = p2[1] = p1[2]*normal[2]/normal[1]
        if normal[1] > 0.0000001:
            p1[1] -= 200*normal[0]/normal[1]
        p1[0] = 200
        if normal[1] > 0.0000001:
            p2[1] -= 200*normal[0]/normal[1]
        p2[0] = -200

        p1 = self.project (cameraId, p1)
        p2 = self.project (cameraId, p2)

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        if abs(dx) <= 1:
            if abs(dy) <= 1:
                raise Exception ("Degenerated camera")
            return InnerModelVector.vec3d (-1, 0, p1[0])
        else:
            h = camera.camera.getHeight()
            return InnerModelVector.vec3d (dy/dx, -1, h - (p1[1]-(dy*p1[0]/dx)) + heightOffset)

    def getHomographyMatrix (self, destCamera: str, planeId: str, sourceCamera: str):
        planeN = self.innerModel.getNode (planeId).normal
        planeN = self.innerModel.getRotationMatrixTo (sourceCamera, planeId).dot (planeN)
        planePoint = self.innerModel.transform (sourceCamera, planeId,
                                                self.innerModel.getNode(planeId).point)
        R = self.innerModel.getRotationMatrixTo (destCamera, sourceCamera)
        t = self.innerModel.transform (destCamera, sourceCamera, InnerModelVector.vec3d (0, 0, 0))
        n = InnerModelMatrix.makeDiagonal (planeN)
        K1 = self.innerModel.getNode (sourceCamera).camera.qmat
        K2 = self.innerModel.getNode (destCamera).camera.qmat

        d = -1*(planePoint.dot(planeN))
        H = K2.dot(R - ((t.dot(n.getTranspose()))/d)).dot(K1.invert())
        return H

    def getAffineHomographyMatrix (self, destCamera: str, planeId: str, sourceCamera: str):
        planeN = self.innerModel.getNode (planeId).normal
        planeN = self.innerModel.getRotationMatrixTo (sourceCamera, planeId).dot (planeN)
        planePoint = self.innerModel.transform (sourceCamera, planeId,
                                                self.innerModel.getNode(planeId).point)
        R = self.innerModel.getRotationMatrixTo (destCamera, sourceCamera)
        t = self.innerModel.transform (destCamera, sourceCamera, InnerModelVector.vec3d (0, 0, 0))
        n = InnerModelMatrix.makeDiagonal (planeN)
        K1 = self.innerModel.getNode (sourceCamera).camera.qmat

        d = -1*(planePoint.dot(planeN))
        H = (R - ((t.dot(n.getTranspose()))/d)).dot(K1.invert())
        for r in range (2):
            for c in range (3):
                H[r][c] = H[r][c]*1000
        return H

    def getPlaneProjectionMatrix (self, destCamera: str, planeId: str, sourceCamera: str):
        planeN = self.innerModel.getNode (planeId).normal
        planeN = self.innerModel.getRotationMatrixTo (sourceCamera, planeId).dot (planeN)
        planePoint = self.innerModel.transform (sourceCamera, planeId,
                                                self.innerModel.getNode(planeId).point)
        R = self.innerModel.getRotationMatrixTo (destCamera, sourceCamera)
        t = self.innerModel.transform (destCamera, sourceCamera, InnerModelVector.vec3d (0, 0, 0))
        n = InnerModelMatrix.makeDiagonal (planeN)
        K1 = self.innerModel.getNode (sourceCamera).camera.qmat

        d = -1*(planePoint.dot(planeN))
        H = (R - ((t.dot(n.getTranspose()))/d)).dot(K1.invert())

        HFinal = InnerModelMatrix ((4,3))
        HFinal.inject (H, 0, 0)
        HFinal = HFinal*1000*1000
        HFinal[3][0] = 1000*H[2][0]
        HFinal[3][1] = 1000*H[2][1]
        HFinal[3][2] = 1000*H[2][2]

        return HFinal

    # declared but not defined in CPP
    def updateStereoGeometry (self, firstCam: str, secondCam: str):
        pass

    # declared but not defined in CPP
    def compute3DPointInCentral (self, firstCam: str, left: 'InnerModelVector', secondCam: str,
                                 right: 'InnerModelVector'):
        pass

    # declared but not defined in CPP
    def compute3DPointInRobot (self, firstCam: str, left: 'InnerModelVector', secondCam: str,
                                 right: 'InnerModelVector'):
        pass

    def compute3DPointFromImageCoords (self, firstCam: str, left: 'InnerModelVector', secondCam: str,
                                 right: 'InnerModelVector', refSystem: str) -> 'InnerModelVector':
        ray = self.backProject (firstCam, left)
        pI = self.innerModel.getRotationMatrixTo (refSystem, firstCam).dot(ray)
        pI[0] = pI[0]/pI[2]
        pI[1] = pI[1]/pI[2]
        pI[2] = 1

        ray = self.backProject (secondCam, right)
        pD = self.innerModel.getRotationMatrixTo (refSystem, secondCam).dot(ray)
        pD[0] = pD[0]/pD[2]
        pD[1] = pD[1]/pD[2]
        pD[2] = 1

        n = np.bitwise_xor (pI, pD)

        A = InnerModelMatrix ((3,3))
        A[0][0] = pI[0]; A[0][1] = -1*pD[0]; A[0][2] = n[0]
        A[1][0] = pI[1]; A[1][1] = -1*pD[1]; A[1][2] = n[1]
        A[2][0] = pI[2]; A[2][1] = -1*pD[2]; A[2][2] = n[2]

        TI = self.innerModel.getRotationMatrixTo (refSystem, firstCam).fromHomogeneousCoordinates()
        TD = self.innerModel.getRotationMatrixTo (refSystem, secondCam).fromHomogeneousCoordinates()
        T = TD - TI

        abc = A.invert().dot(T)

        pR = pI.dot(abc[0])
        pR = pR + TI
        pR = (n.dot(abc[2]/2)) + pR

        return pR

    def compute3DPointFromImageAngles (self, firstCam: str, left: 'InnerModelVector', secondCam: str,
                                       right: 'InnerModelVector', refSystem: str) -> 'InnerModelVector':
        ray = InnerModelVector.vec3d (math.tan(left[0]), math.tan(left[1]), 1)
        pI = copy.deepcopy(ray)

        pI[0] = pI[0]/pI[2]
        pI[1] = pI[1]/pI[2]
        pI[2] = 1

        ray[0] = math.tan (right[0])
        ray[1] = math.tan (right[1])
        ray[2] = 1
        pD = copy.deepcopy (ray)

        pD[0] = pD[0]/pD[2]
        pD[1] = pD[1]/pD[2]
        pD[2] = 1

        n = np.bitwise_xor (pI, pD)

        A = InnerModelMatrix ((3,3))
        A[0][0] = pI[0]; A[0][1] = -1*pD[0]; A[0][2] = n[0]
        A[1][0] = pI[1]; A[1][1] = -1*pD[1]; A[1][2] = n[1]
        A[2][0] = pI[2]; A[2][1] = -1*pD[2]; A[2][2] = n[2]

        TI = self.innerModel.getRotationMatrixTo (refSystem, firstCam).fromHomogeneousCoordinates()
        TD = self.innerModel.getRotationMatrixTo (refSystem, secondCam).fromHomogeneousCoordinates()
        T = TD - TI

        abc = A.invert().dot(T)

        pR = pI.dot(abc[0])
        pR = pR + TI
        pR = (n.dot(abc[2]/2)) + pR

        return pR


