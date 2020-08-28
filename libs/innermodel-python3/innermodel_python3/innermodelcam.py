import copy
import math

from innermodelmatrix import InnerModelMatrix
from innermodelvector import InnerModelVector

class InnerModelCam (object):
    def __init__ (self, Fx: float = None, Fy: float = None, Ox: float = None, Oy: float = None,
                  c: 'InnerModelCam' = None):
        self.qmat = InnerModelMatrix ((3,3))
        if (c is not None):
            self = copy.deepcopy (c)
        elif Fx is not None:
            self.focalX = Fx
            self.focalY = Fy
            self.centerX = Ox
            self.centerY = Oy
            self.width = Ox*2
            self.height = Oy*2
            self.size = self.width*self.height
            self.qmat[0][0] = Fx; self.qmat[0][1] = 0; self.qmat[0][1] = Ox
            self.qmat[1][0] = 0; self.qmat[1][1] = -1*Fy; self.qmat[1][2] = Oy
            self.qmat[2][0] = 0; self.qmat[2][1] = 0; self.qmat[2][2] = 1
        else:
            self.focalX = 200
            self.focalY = 200
            self.centerX = 160
            self.centerY = 120
            self.width = self.centerX*2
            self.height = self.centerY*2
            self.size = self.width*self.height
        self.focusX = None
        self.focusY = None

    def getAngles (self, p: 'InnerModelVector') -> 'InnerModelVector':
        assert (p.size() == 2)
        r = self.toZeroCenter(p)
        r[0] = math.atan2 (r[0], self.focusX)
        r[1] = math.atan2 (r[1], self.focusY)
        return r

    def getAnglesHomogeneous (self, p: 'InnerModelVector') -> 'InnerModelVector':
        r = self.toZeroCenterHomogeneous(p)
        r[0] = r[0]/self.focusX
        r[1] = r[1]/self.focusY
        return r

    def getFocal (self) -> float:
        return self.focusX

    def getFocalX (self) -> float:
        return self.focalX

    def getFocalY (self) -> float:
        return self.focalY

    def getHeight (self) -> int:
        return self.height

    def getWidth (self) -> int:
        return self.width

    def getSize (self) -> int:
        return self.size

    def getRayHomogeneous (self, p: 'InnerModelVector') -> 'InnerModelVector':
        r = self.toZeroCenterHomogeneous (p)
        r[0] = r[0]/self.focusX
        r[1] = r[1]/self.focusY
        return r

    def getRay (self, p: 'InnerModelVector') -> 'InnerModelVector':
        r = self.toZeroCenter (p)
        r[0] = r[0]/self.focusX
        r[1] = r[1]/self.focusY
        return r

    def polar3DToCamera (self, p: 'InnerModelMatrix') -> 'InnerModelMatrix':
        r = InnerModelMatrix ((3,1))
        r[2] = p[2]/(p[0]*p[0]/(self.focusX*self.focusY) + (p[1]*p[1]/(self.focusX*self.focusY) + 1))
        r[0] = r[2]*p[0]/self.focusX
        r[1] = r[2]*p[1]/self.focusY

        return r

    def project (self, p: 'InnerModelVector') -> 'InnerModelVector':
        loc = self.qmat.dot(p)
        res = InnerModelVector ((3,))
        if (abs(loc[2]) != 0):
            res[0] = loc[0]/loc[2]
            res[1] = loc[1]/loc[2]
            res[2] = loc[2]
            return res
        else:
            print ("InnerModelCamera: warning projected point at infinite")
            return res

    def set (self, Fx: float, Fy: float, Ox: float, Oy: float):
        self.focalX = Fx
        self.focalY = Fy
        self.centerX = Ox
        self.centerY = Oy
        self.width = Ox*2
        self.height = Oy*2
        self.size = self.width*self.height
        self.qmat[0][0] = Fx; self.qmat[0][1] = 0; self.qmat[0][1] = Ox
        self.qmat[1][0] = 0; self.qmat[1][1] = -1*Fy; self.qmat[1][2] = Oy
        self.qmat[2][0] = 0; self.qmat[2][1] = 0; self.qmat[2][2] = 1

    def setFocal (self, f: float):
        self.focusX = f
        self.focusY = f

    def setFocalX (self, fx: float):
        self.focalX = fx

    def setFocalY (self, fy: float):
        self.focalY = fy

    def setSize (self, w: int, h: int):
        self.width = w
        self.height = h
        self.size = w*h

    def toZeroCenter (self, p: 'InnerModelVector') -> 'InnerModelVector':
        assert (p.size() >= 2)
        r = InnerModelVector ((2,))
        r[0] = p[0] - self.centerX
        r[1] = p[1] - self.centerY
        return r

    def toZeroCenterHomogeneous (self, p: 'InnerModelVector') -> 'InnerModelVector':
        assert (p.size() >= 2)
        r = InnerModelVector ((3,))
        r[0] = p[0] - self.centerX
        r[1] = r[1] - self.centerY
        r[2] = 1
        return r


