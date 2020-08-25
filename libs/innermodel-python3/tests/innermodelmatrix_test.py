import unittest
import sys
import numpy as np

sys.path.append('../innermodel_python3')

from innermodelmatrix import InnerModelMatrix
from innermodelvector import InnerModelVector
from scipy.spatial.transform import Rotation as R

def isDiag(M):
    i, j = np.nonzero(M)
    return np.all(i == j)

class TestInnerModelMat (unittest.TestCase):

    def testSVD(self):
        m1 = InnerModelMatrix((3,3))
        self.assertTrue (m1 is not None)
        m = np.array([[10,3,4],
                    [0,1,0],
                    [0,10,90]])
        np.copyto (m1, m)
        U, S, V = m1.SVD()
        U_T = U.getTranspose()
        V_T = V.getTranspose()
        I = np.identity(3)

        self.assertTrue(np.allclose (U.dot(U_T), I))
        self.assertTrue(np.allclose (V.dot(V_T), I))

        self.assertTrue (isDiag(S))

    def testInvert (self):
        m1 = InnerModelMatrix((3,3))
        m = np.array([[10,0,0],
                    [5,10,0],
                    [0,0,10]])
        np.copyto (m1, m)
        m1_inv = m1.invert()
        I = np.identity(3)
        self.assertTrue (np.allclose (m1.dot(m1_inv), I))

        m = np.array ([[4,0,0,0],
                       [0,4,0,0],
                       [0,0,4,0]])
        m1 = InnerModelMatrix((3,4))
        np.copyto (m1, m)
        with self.assertRaises(AssertionError):
            m1_inv = m1.invert()

    def testEuler (self):
        m1 = InnerModelMatrix((3,3))
        r = R.from_euler ('xyz', [0, 0, 90], degrees=True)
        m2 = r.as_matrix()
        np.copyto (m1, m2)
        v = m1.extractAnglesR()
        self.assertTrue (np.allclose (v, np.array([0, 0, np.pi/2])))

    def testEigen (self):
        m = InnerModelMatrix((3,3))
        m1 = np.array([[2,3,0],
                        [0,3,8],
                        [0,0,1]])

        np.copyto (m, m1)
        w, V = np.linalg.eig(m1)
        w_, V_ = m.eigenValsVectors()
        self.assertTrue (np.allclose(w, w_))
        self.assertTrue (np.allclose(V, V_))

    # TODO: check makeDefPos, it actually makes the matrix semi positive definite
    def testDefPos (self):
        pass

    # TODO: check cholesky, eigenValsValues, invert (if actually inverts or not)
    def testCholesky (self):
        pass

    # TODO: check unitModulus function (if actually makes the modulus 1)
    def testUnitModulus (self):
        pass

    # TODO
    def testMatSqrt (self):
        pass

if __name__ == '__main__':
    unittest.main()

