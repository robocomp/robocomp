'''
Classes for Rotation Matrices: Rx, Ry, Rz, R = Rx x Ry x Rz
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
import numpy as np
from innermodelmatrix import InnerModelMatrix

class Rot3DOnAxis(InnerModelMatrix):
    '''Base class for all the rotation matrices'''

    def __new__(cls, *args, **kwargs):
        return super(Rot3DOnAxis, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        '''Takes the same argument as numpy.ndarray'''

        self.ang = 0

    def update (self, alpha: float):
        '''To implemented by the derived classes'''

        pass

    def getAlpha (self) -> float:
        '''Returns the angle of rotation of the rotation matrix'''

        return self.ang

class Rot3DOX(Rot3DOnAxis):
    '''3D Rotation matrix around the x axis CCW'''

    @staticmethod
    def getRot3DOX(alpha: float = 0, m : 'Rot3DOX' = None) -> 'Rot3DOX':
        '''Resturns an instantiation of 'Rot3DOX' class with the given angle of rotation'''

        if m is not None:
            a = m.getAlpha()
            x = Rot3DOX((3,3))
            np.copyto(x, m)
            x.update (a)
            return x

        x = Rot3DOX((3,3))
        x.update(alpha)
        x[0][0] = 1
        x[0][1] = 0
        x[0][2] = 0
        x[1][0] = 0
        x[2][0] = 0
        return x

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[1][1] = math.cos(alpha)
        self[1][2] = -math.sin(alpha)
        self[2][1] = math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# redundant
class Rot3DCOX(Rot3DOnAxis):
    '''3D Rotation matrix around the x axis CW'''

    @staticmethod
    def getRot3DCOX(alpha: float = 0, m : 'Rot3DCOX'= None) -> 'Rot3DCOX':
        '''Returns an instantion of 'Rot3DCOX with the given angle of roation'''

        if m is not None:
            alpha = m.getAlpha()
            x = Rot3DCOX((3,3))
            np.copyto(x, m)
            x.update (alpha)
            return x

        x = Rot3DCOX((3,3))
        x.update(alpha)
        x[0][0] = 1
        x[0][1] = 0
        x[0][2] = 0
        x[1][0] = 0
        x[2][0] = 0
        return x

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[1][1] = math.cos(alpha)
        self[1][2] = math.sin(alpha)
        self[2][1] = -math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

class Rot3DOY(Rot3DOnAxis):
    '''3D Rotation matrix around the y axis CCW'''

    @staticmethod
    def getRot3DOY(alpha: float = 0, m : 'Rot3DOY' = None) -> 'Rot3DOY':
        '''Returns an instantion of 'Rot3DOY with the given angle of roation'''

        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DOY((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        y = Rot3DOY((3,3))
        y.update(alpha)
        y[0][1] = 0
        y[1][0] = 0
        y[1][1] = 1
        y[1][2] = 0
        y[2][1] = 0
        return y

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[0][0] = math.cos(alpha)
        self[0][2] = math.sin(alpha)
        self[2][0] = -math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

# redundant
class Rot3DCOY(Rot3DOnAxis):
    '''3D Rotation matrix around the x axis CW'''

    @staticmethod
    def getRot3DCOY(alpha: float = 0, m : 'Rot3DCOY' = None) -> 'Rot3DCOY':
        '''Returns an instantion of 'Rot3DCOY with the given angle of roation'''

        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DCOY((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        y = Rot3DCOY((3,3))
        y.update(alpha)
        y[0][1] = 0
        y[1][0] = 0
        y[1][1] = 1
        y[1][2] = 0
        y[2][1] = 0
        return y

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[0][0] = math.cos(alpha)
        self[0][2] = -math.sin(alpha)
        self[2][0] = math.sin(alpha)
        self[2][2] = math.cos(alpha)
        self.ang = alpha

class Rot3DOZ(Rot3DOnAxis):
    '''3D Rotation matrix around the z axis CCW'''

    @staticmethod
    def getRot3DOZ(alpha: float = 0, m : 'Rot3DOZ' = None) -> 'Rot3DOZ':
        '''Returns an instantion of 'Rot3DOZ with the given angle of roation'''

        if m is not None:
            alpha = m.getAlpha()
            z = Rot3DOZ((3,3))
            np.copyto(z, m)
            z.update (alpha)
            return z

        z = Rot3DOZ((3,3))
        z.update(alpha)
        z[0][2] = 0
        z[1][2] = 0
        z[2][0] = 0
        z[2][1] = 0
        z[2][2] = 1
        return z

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[0][0] = math.cos(alpha)
        self[0][1] = -math.sin(alpha)
        self[1][0] = math.sin(alpha)
        self[1][1] = math.cos(alpha)
        self.ang = alpha

# redundant
class Rot3DCOZ(Rot3DOnAxis):
    '''3D Rotation matrix around the x axis CW'''

    @staticmethod
    def getRot3DCOZ(alpha: float = 0, m : 'Rot3DCOZ' = None) -> 'Rot3DCOZ':
        '''Returns an instantion of 'Rot3DCOZ with the given angle of roation'''

        if m is not None:
            alpha = m.getAlpha()
            y = Rot3DCOZ((3,3))
            np.copyto(y, m)
            y.update (alpha)
            return y

        z = Rot3DCOZ((3,3))
        z.update(alpha)
        z[0][2] = 0
        z[1][2] = 0
        z[2][0] = 0
        z[2][1] = 0
        z[2][2] = 1
        return z

    def update (self, alpha: float):
        '''Update the matrix with change in angle of rotaion'''

        self[0][0] = math.cos(alpha)
        self[0][1] = math.sin(alpha)
        self[1][0] = -math.sin(alpha)
        self[1][1] = math.cos(alpha)
        self.ang = alpha

class Rot3D(InnerModelMatrix):
    '''Rotation matrix CCW'''

    @staticmethod
    def getRot3D (ox: float = 0, oy: float = 0, oz: float = 0, ex = None) -> 'Rot3D':
        '''
        Returns an instantion of the class 'Rot3D' with
        the given angle of rotation around all the axis
        '''

        if ex is not None:
            mat = Rot3D(ex.shape)
            np.copyto (mat, ex)

            np.copyto(mat.RX, ex.RX)
            np.copyto(mat.RY, ex.RY)
            np.copyto(mat.RZ, ex.RZ)

            return mat
        else:
            mat = Rot3D((3,3))
            mat.RX = Rot3DOX.getRot3DOX(ox)
            mat.RY = Rot3DOY.getRot3DOY(oy)
            mat.RZ = Rot3DOZ.getRot3DOZ(oz)
            _m  = mat.RX.dot(mat.RY.dot(mat.RZ))
            np.copyto(mat, _m)
            return mat

    def __new__(cls, *args, **kwargs):
        return super(Rot3D, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        '''Initialisation of rotation matrices with the angle 0'''

        self.RX = Rot3DOX.getRot3DOX(0)
        self.RY = Rot3DOY.getRot3DOY(0)
        self.RZ = Rot3DOZ.getRot3DOZ(0)

    def update (self, ox: float, oy: float, oz: float):
        '''Update the rotation matrices with the new rotation angles'''

        self.RX.update(ox)
        self.RY.update(oy)
        self.RZ.update(oz)
        _m  = self.RX.dot(self.RY.dot(self.RZ))
        np.copyto(self, _m)

# redundant
class Rot3DC(InnerModelMatrix):
    '''Rotation matrix CW'''

    @staticmethod
    def getRot3DC (ox: float = 0, oy: float = 0, oz: float = 0, ex = None) -> 'Rot3DC':
        '''
        Returns an instantion of the class 'Rot3DC' with
        the given angle of rotation around all the axis
        '''

        if ex is not None:
            mat = Rot3DC(ex.shape)
            np.copyto (mat, ex)

            np.copyto (mat.RX, ex.RX)
            np.copyto (mat.RY, ex.RY)
            np.copyto (mat.RZ, ex.RZ)

            return mat
        else:
            mat = Rot3DC((3,3))
            mat.RX = Rot3DOX.getRot3DOX(ox)
            mat.RY = Rot3DOY.getRot3DOY(oy)
            mat.RZ = Rot3DOZ.getRot3DOZ(oz)
            _m  = mat.RX.dot(mat.RY.dot(mat.RZ))
            np.copyto(mat, _m)
            return mat

    def __new__(cls, *args, **kwargs):
        return super(Rot3DC, cls).__new__(cls, *args, **kwargs)

    def __init__(self, *args, **kwargs):
        '''Initialisation of the matrices with angle of rotation 0'''

        self.RX = Rot3DCOX.getRot3DCOX(0)
        self.RY = Rot3DCOY.getRot3DCOY(0)
        self.RZ = Rot3DCOZ.getRot3DCOZ(0)

    def update (self, ox: float, oy: float, oz: float):
        '''Update the rotation matrices with new angle of rotation'''

        self.RX.update(ox)
        self.RY.update(oy)
        self.RZ.update(oz)
        _m  = self.RX.dot(self.RY.dot(self.RZ))
        np.copyto(self, _m)

class Rot2D(InnerModelMatrix):
    '''rotation in a plane CCW'''

    @staticmethod
    def getRot2D(alpha: float) -> 'Rot2D':
        '''Returns an instantiation of 2D rotation matrix with the given angle'''

        mat = Rot2D((2,2))
        mat[0][0] = math.cos(alpha)
        mat[0][1] = math.sin(alpha)
        mat[1][0] = -math.sin(alpha)
        mat[1][1] = math.cos(alpha)
        return mat

    def update (self, alpha: float):
        '''Update the roation matrix with new angle of rotation'''

        self[0][0] = math.cos(alpha)
        self[0][1] = math.sin(alpha)
        self[1][0] = -math.sin(alpha)
        self[1][1] = math.cos(alpha)

# redundant
class Rot2DC(InnerModelMatrix):
    '''rotation in a plane CW'''

    @staticmethod
    def getRot2DC(alpha: float) -> 'Rot2DC':
        '''Returns an instantiation of 2D roation with the given angle'''

        mat = Rot2DC((2,2))
        mat[0][0] = math.cos(alpha)
        mat[0][1] = -math.sin(alpha)
        mat[1][0] = math.sin(alpha)
        mat[1][1] = math.cos(alpha)
        return mat

    def update (self, alpha: float):
        '''Update the rotation matrix with new angle of rotation'''

        self[0][0] = math.cos(alpha)
        self[0][1] = -math.sin(alpha)
        self[1][0] = math.sin(alpha)
        self[1][1] = math.cos(alpha)
