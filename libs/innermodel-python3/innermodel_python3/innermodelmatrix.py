'''
2D matrix class for InnerModel. Wrapper around numpy.ndarray
Author: ksakash@github.com (Akash Kumar Singh)
'''

import math
import numpy as np
from innermodelvector import InnerModelVector
from scipy.spatial.transform import Rotation as R

class InnerModelMatrix(np.ndarray):
    '''
    2D matrix class for InnerModel. Wrapper around numpy.ndarray
    Note: most of the methods are written specifically for a square matrix
    '''

    @staticmethod
    def Mat (m: int, n: int) -> 'InnerModelMatrix':
        '''Returns 2D Matrix of dimension: m x n'''

        matrix = InnerModelMatrix((m,n))
        return matrix

    @staticmethod
    def makeDiagonal (v: 'InnerModelVector') -> 'InnerModelMatrix':
        '''Returns a diagonal matrix with diagonal element equal to elements of vector \'v\''''

        x = np.diag (v)
        y = InnerModelMatrix(x.shape)
        np.copyto(y,x)
        return y

    @staticmethod
    def identity (m: int) -> 'InnerModelMatrix':
        '''Returns an identity matrix of dimension \'m\''''

        y = InnerModelMatrix((m,m))
        np.copyto(y, np.identity(m))
        return y

    @staticmethod
    def diagonalMat (m: 'InnerModelMatrix') -> 'InnerModelMatrix':
        '''
        Returns a diagonal matrix with diagonal element equal to diagonal element of matrix 'm'
        '''

        assert (m.shape[0] == m.shape[1])
        d = m.diagonal()
        x = np.diag(d)
        y = InnerModelMatrix(m.shape)
        np.copyto(y, x)
        return y

    @staticmethod
    def ones (m: int, n: int) -> 'InnerModelMatrix':
        '''
        Returns a 2D matix of dimension (m x n) with all the element equal to 1
        '''
        x = np.ones ((m,n))
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    @staticmethod
    def random (m: int, n: int) -> 'InnerModelMatrix':
        '''
        Returns a 2D matrix of dimension (m x n) will all the elements between 0 to 1
        '''

        x = np.random.rand (m,n)
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    @staticmethod
    def zeroes (m: int, n: int) -> 'InnerModelMatrix':
        '''
        Returns a 2D matix of dimension (m x n) with all the element equal to 0
        '''

        x = np.zeros ((m,n))
        y = InnerModelMatrix((m,n))
        np.copyto(y,x)
        return y

    @staticmethod
    def equalSize (first: 'InnerModelMatrix', second: 'InnerModelMatrix') -> bool:
        '''Returns a bool whether 'first' vector and 'second' vector are equal'''

        return (first.shape[0] == second.shape[0] and first.shape[1] == second.shape[1])

    def set (self, value: float):
        '''Sets all the element equal to \'value\''''

        self.fill(value)

    def copy (self) -> 'InnerModelMatrix':
        '''Returns a copy of the matrix'''

        mat = InnerModelMatrix(self.shape)
        np.copyto(mat, self)
        return mat

    def inject (self, matrix: 'InnerModelMatrix', roff: float, coff: float) -> 'InnerModelMatrix':
        '''Inject a matrix at the given row and col offset'''

        rows = matrix.shape[0]
        cols = matrix.shape[1]
        assert (rows+roff <= self.shape[0] and cols+coff <= self.shape[1])
        self[roff:roff+rows, coff:coff+cols] = matrix
        mat = InnerModelMatrix(self.shape)
        np.copyto(mat, self)
        return mat

    def fillDiagonal (self, value: float):
        '''Change all the diagonal elements to \'vallue\''''

        np.fill_diagonal(self, value)

    def getDiagonal (self) -> 'InnerModelVector':
        '''Returns diagonal elements in form of a vector'''

        x = self.diagonal()
        y = InnerModelVector((x.shape[0]))
        np.copyto(y, x)
        return y

    def getTranspose (self) -> 'InnerModelMatrix':
        '''Returns transpose of the given matrix'''

        return self.transpose()

    def determinant (self) -> float:
        '''Returns determinant of the matrix'''

        assert (self.shape[0] == self.shape[1])
        return np.linalg.det(self)

    def trace (self) -> float:
        '''Returns trace of the matrix'''

        assert (self.shape[0] == self.shape[1])
        m = np.array(self)
        return np.trace(m)

    def invert (self) -> 'InnerModelMatrix':
        '''Returns inverse of the matrix if non-singular or pseudo-inverse if singular'''

        assert(self.isSquare())
        if (self.determinant() != 0):
            m = np.linalg.inv(self)
            mat = InnerModelMatrix(m.shape)
            np.copyto(mat, m)
            return mat
        else:
            m = np.linalg.pinv(self)
            mat = InnerModelMatrix(m.shape)
            np.copyto(mat, m)
            return mat

    def fillOnes (self):
        '''set all the elements equal to 1'''

        self.set(1)

    def makeUnitModulus (self) -> 'InnerModelMatrix':
        '''Normalizes the matrix'''

        assert(self.isColVector())
        mod = self.vectorNormL2()
        if (mod > 0):
            self = self/mod
        y = InnerModelMatrix(self.shape)
        np.copyto(y, self)
        return y

    def makeIdentity (self):
        '''Convert the matrix to an identity matrix'''

        assert(self.shape[0] == self.shape[1])
        x = np.identity(self.shape[0])
        np.copyto(self, x)

    def isSquare (self, other: 'InnerModelMatrix' = None) -> bool:
        '''Checks if it is a square matrix'''

        if other is not None:
            return (other.shape[0] == other.shape[1])
        return (self.shape[0] == self.shape[1])

    def is3ColumnVector (self, other: 'InnerModelMatrix' = None) -> bool:
        '''Checks if it has 3 columns'''

        if other is not None:
            return (other.shape[0] == 3 and other.shape[1] == 1)
        return (self.shape[0] == 3 and self.shape[1] == 1)

    def canAllocateRotationMatrix (self) -> bool:
        '''Checks if we can make it the rotation matrix'''

        return (self.shape[0] >= 3 and self.shape[1] >= 3)

    def isColVector (self) -> bool:
        '''Checks if the matrix is a column vector'''

        return (self.shape[0] > 0 and self.shape[1] == 1)

    def isEmpty (self) -> bool:
        '''Returns if the matrix is empty'''

        return not (self.size > 0)

    def minDim (self, other: 'InnerModelMatrix' = None) -> int:
        '''Returns smaller dimension of the matrix'''

        if (other is not None):
            return min (other.shape[0], other.shape[1])
        return min (self.shape[0], self.shape[1])

    def maxDim (self, other: 'InnerModelMatrix' = None) -> int:
        '''Returns greater dimension of the matrix'''

        if (other is not None):
            return max (other.shape[0], other.shape[1])
        return max (self.shape[0], self.shape[1])

    def sqrt (self) -> 'InnerModelMatrix':
        '''Returns a matrix with element equal to sqare root of element of the given matrix'''

        _abs = np.absolute (self)
        return np.sqrt (_abs)

    def isPosDef (self) -> bool:
        '''Check if the matrix is positive definite'''

        assert (self.isSquare())
        return np.all(np.linalg.eigvals(self) > 0)

    def cholesky (self) -> 'InnerModelMatrix':
        '''Perform cholesky decomposition'''

        assert (self.isPosDef())
        return np.linalg.cholesky(self)

    def eigenValsVectors (self) -> ('InnerModelVector', 'InnerModelMatrix'):
        '''Returns eigen values and eigen vectors'''

        assert (self.isSquare())
        w, V = np.linalg.eig (self)
        y = InnerModelVector(w.shape)
        np.copyto (y, w)
        return (y, V)

    def SVD (self) -> ('InnerModelMatrix', 'InnerModelMatrix', 'InnerModelMatrix'):
        '''Perform singular value decomposition'''

        U, _S, V_T = np.linalg.svd (self, full_matrices=True)
        S = InnerModelMatrix.zeroes (self.shape[0], self.shape[1])
        size = _S.shape[0]
        for i in range (size):
            S[i][i] = _S[i]
        return (U, S, V_T.getTranspose())

    def makeDefPos (self) -> 'InnerModelMatrix':
        '''Convert the matrix to definite positive'''

        (w, V) = self.eigenValsVectors()
        size = w.shape[0]
        for i in range(size):
            if w[i] <= 0:
                w[i] = 0.000001
        DD = InnerModelMatrix.makeDiagonal(w)
        return (V*DD*V.getTranspose())

    # TODO
    def matSqrt (self) -> 'InnerModelMatrix':
        '''Returns square root of the matrix'''

        assert (self.isSquare())
        (_, V) = self.eigenValsVectors()
        V_I = V.invert()
        _D = V_I * self * V
        D = InnerModelMatrix.zeroes(_D.shape[0], _D.shape[1])
        size = D.shape[0]
        for i in range (size):
            D[i][i] = math.sqrt(abs(_D[i][i]))
        R = V*D*V_I
        return R

    def vectorNormL2 (self) -> float:
        '''Returns L2 norm of the matrix'''

        return np.linalg.norm(self)

    def toVector (self) -> 'InnerModelVector':
        '''Converts the 1 column matrix to a vector'''

        assert (self.shape[0] > 0 and self.shape[1] == 1)
        rows = self.shape[0]
        y = InnerModelVector((rows,))
        np.copyto (y, self[:, 0])
        return y

    # Eulers angles from rotation matrix in radians
    def extractAnglesR(self) -> 'InnerModelMatrix':
        '''Returns euler angles from the rotation matrix'''

        assert (self.shape == (3,3))
        r = R.from_matrix(self)
        _v = r.as_euler('xyz')
        v = InnerModelMatrix(_v.shape)
        np.copyto(v, _v)
        return v

    # Choosing the one which has lower norm
    def extractAnglesR_min (self) -> 'InnerModelVector':
        '''Returns euler angles with lower magnitude'''

        r = self.extractAnglesR()
        if (r.shape[0] == 1):
            return InnerModelVector.vec3d(r[0], r[1], r[2])
        v1 = InnerModelVector((3,))
        np.copyto(v1, r[0])
        v2 = InnerModelVector((3,))
        np.copyto(v2, r[1])
        if (np.linalg.norm(v1) < np.linalg.norm(v2)):
            return v1
        return v2

    def setCol (self, col: int, vector: 'InnerModelVector'):
        '''Sets a particular column with the given vector'''

        assert (self.shape[0] == vector.shape[0])
        np.copyto (self[:,col], vector)

    def getCol (self, col: int) -> 'InnerModelVector':
        '''Returns a give column'''

        y = InnerModelVector ((self.shape[0],))
        np.copyto (y, self[:,col])
        return y

    def setRow (self, row: int, vector: 'InnerModelVector'):
        '''Sets a particular row with the given vector'''

        assert (self.shape[1] == vector.shape[0])
        np.copyto (self[row,:], vector)

    def getRow (self, row: int) -> 'InnerModelVector':
        '''Get a particular row'''

        y = InnerModelVector ((self.shape[1],))
        np.copyto (y, self[row,:])
        return y

    def getSubmatrix (self, firstRow: int, lastRow: int, firstCol: int,
                                                                lastCol: int) -> 'InnerModelMatrix':
        '''Returns the matrix within the given range'''

        return self[firstRow:lastRow+1, firstCol:lastCol+1]
