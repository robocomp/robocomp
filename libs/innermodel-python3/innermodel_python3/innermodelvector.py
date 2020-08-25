'''
1D vector for InnerModel. Wrapper around numpy.ndarray
Author: ksakash@github.com (Akash Kumar Singh)
'''

import numpy as np

class InnerModelVector(np.ndarray):
    '''1D vector wrapped around numpy.ndarray'''

    @staticmethod
    def vec3d(x: float, y: float, z: float) -> 'InnerModelVector':
        '''1D vector of length 3'''

        ret = InnerModelVector((3,))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        return ret

    @staticmethod
    def vec6d(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> 'InnerModelVector':
        '''1D vector of length 6'''

        ret = InnerModelVector((6,))
        ret[0] = x
        ret[1] = y
        ret[2] = z
        ret[3] = rx
        ret[4] = ry
        ret[5] = rz
        return ret

    def size (self) -> int:
        '''Returns the size of 1D vector'''

        return self.shape[0]

    def set(self, val: float):
        '''Sets the value of all the elements of the vector to be equal to \'val\''''

        self.fill(val)

    def subvector(self, firstIndex: int, lastIndex: int) -> 'InnerModelVector':
        '''Returns a vector with all the elements from 'firstIndex' to \'lastIndex\''''

        assert(firstIndex >= 0)
        assert(firstIndex <= lastIndex)
        return self[firstIndex : lastIndex]

    def scalar_division(self, val: float) -> 'InnerModelVector':
        '''Divides elements of the vector with \'val\''''

        assert (val != 0)
        return (self/val)

    def scalar_multiplication(self, val: float) -> 'InnerModelVector':
        '''Mulitplies elements of the vector with \'val\''''

        return self*val

    def inject(self, other: 'InnerModelVector', offset: int):
        '''Inject 'other' vector at an offset from the 0th index'''

        assert(other.shape[0] + offset <= self.shape[0])
        size = other.shape[0]
        self[offset:offset+size] = other

    def point_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        '''
        Given: self: [a1...an], other: [b1...bn]
        Return: [a1*b1...an*bn]
        '''

        return self*other

    def normalized(self) -> 'InnerModelVector':
        '''
        Make the magnitude of the vector to be one by dividing the elements with the magnitude
        '''

        return self.scalar_division(self.norm2())

    def cross_product(self, other: 'InnerModelVector') -> 'InnerModelVector':
        '''Returns cross product of two vector: self x other'''

        return np.cross(self, other)

    def equals(self, other: 'InnerModelVector') -> bool:
        '''Returns a bool if self & other are equal'''

        return np.array_equal(self, other)

    def is_zero(self) -> bool:
        '''Returns a bool if all the elements are zero'''

        size = self.shape[0]
        return np.array_equal(self, np.zeros(size))

    def toHomogeneousCoordinates(self) -> 'InnerModelVector':
        '''Returns the vector to homogeneous coordinates'''

        return np.append(self, 1.0)

    def fromHomogeneousCoordinates(self) -> 'InnerModelVector':
        '''Returns the vector from homogeneous coordinates'''

        size = self.shape[0]
        last = self[size-1]
        self = np.resize(self, size-1)
        assert(last != 0)
        return (self/last)

    def norm2(self) -> float:
        '''Returns L2 norm of the vector'''

        return np.linalg.norm(self)

    def min(self) -> [float, int]:
        '''Returns minimum element as well as the index of the element'''

        _i = np.argmin(self)
        _m = self[_i]
        return [_m, _i]

    def max(self) -> [float, int]:
        '''Returns maximum element as well as the index of the element'''

        _i = np.argmax(self)
        _m = self[_i]
        return [_m, _i]

    def min_abs(self) -> [float, int]:
        '''Returns minimum element of the absolute vector as well as the index of the element'''

        _abs = np.absolute(self)
        _i = np.argmin(_abs)
        _m = _abs[_i]
        return [_m, _i]

    def max_abs(self) -> [float, int]:
        '''Returns maximum element of the absolute vector as well as the index of the element'''

        _abs = np.absolute(self)
        _i = np.argmax(_abs)
        _m = _abs[_i]
        return [_m, _i]

    def mean(self) -> float:
        '''Returns mean of all the elements in the vector'''

        return np.mean(self)

    def variance(self) -> float:
        '''Returns variance of all the elements in the vector'''

        return np.var(self)

    def dot_product(self, other: 'InnerModelVector') -> float:
        return np.dot(self, other)

    def x(self):
        '''Returns first element of the vector'''

        assert(self.shape[0] >= 3)
        return self[0]

    def y(self):
        '''Returns second element of the vector'''

        assert(self.shape[0] >= 3)
        return self[1]

    def z(self):
        '''Returns third element of the vector'''

        assert(self.shape[0] >= 3)
        return self[2]

    def rx(self):
        '''Returns fourth element of the vector'''

        assert(self.shape[0] >= 6)
        return self[3]

    def ry(self):
        '''Returns fifth element of the vector'''

        assert(self.shape[0] >= 6)
        return self[4]

    def rz(self):
        '''Returns sixth element of the vector'''

        assert(self.shape[0] >= 6)
        return self[5]
