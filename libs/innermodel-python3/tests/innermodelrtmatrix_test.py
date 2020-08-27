import sys
import unittest
import numpy as np

sys.path.append('../innermodel_python3')

from innermodelvector import InnerModelVector
from innermodelrtmatrix import InnerModelRTMat, InnerModelRTCMat
from scipy.spatial.transform import Rotation as R

class TestInnerModelRTMat (unittest.TestCase):

    def testGetRTMat (self):
        mat = InnerModelRTMat.getInnerModelRTMat (tx=10, ty=2, tz=1,
                                                rx=np.pi/4, ry=np.pi/6, rz=np.pi/3)
        self.assertTrue (mat is not None)

        r1 = R.from_euler ('x', 45, degrees=True)
        m = r1.as_matrix()
        self.assertTrue (np.allclose (m, mat.Rx))

        r2 = R.from_euler ('y', 30, degrees=True)
        m = r2.as_matrix()
        self.assertTrue (np.allclose (m, mat.Ry))

        r3 = R.from_euler ('z', 60, degrees=True)
        m = r3.as_matrix()
        self.assertTrue (np.allclose (m, mat.Rz))

        r = r1 * r2 * r3
        m = r.as_matrix()
        self.assertTrue (np.allclose (m, mat.R))

        v = InnerModelVector.vec3d (10, 2, 1)
        self.assertTrue (np.allclose (mat.Tr, v))

    # TODO
    def testInvert (self):
        pass

if __name__ == '__main__':
    unittest.main()
