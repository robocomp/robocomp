import unittest
import numpy as np

import sys

sys.path.append('../innermodel_python3')

from innermodelrotmatrix import Rot3DOX, Rot3DOY, Rot3DOZ
from innermodelrotmatrix import Rot3DCOX, Rot3DCOY, Rot3DCOZ
from innermodelrotmatrix import Rot3D, Rot3DC, Rot2D, Rot2DC

from scipy.spatial.transform import Rotation as R

class TestInnerModelRotMat (unittest.TestCase):

    # TODO : CW
    def testRot3DOX(self):
        ccw1 = Rot3DOX.getRot3DOX(alpha=np.pi/2)
        self.assertTrue (ccw1 is not None)
        ccw2 = Rot3DOX.getRot3DOX(m=ccw1)
        self.assertTrue (ccw2 is not None)

        r = R.from_euler ('xyz', [90, 0, 0], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))
        self.assertTrue (np.allclose (ccw2, m))

        ccw1.update(alpha=np.pi/4)
        r = R.from_euler ('xyz', [45, 0, 0], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))

    # TODO : CW
    def testRot3DOY(self):
        ccw1 = Rot3DOY.getRot3DOY(alpha=np.pi/2)
        self.assertTrue (ccw1 is not None)
        ccw2 = Rot3DOY.getRot3DOY(m=ccw1)
        self.assertTrue (ccw2 is not None)

        r = R.from_euler ('xyz', [0, 90, 0], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))
        self.assertTrue (np.allclose (ccw2, m))

        ccw1.update(alpha=np.pi/4)
        r = R.from_euler ('xyz', [0, 45, 0], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))

    # TODO : CW
    def testRot3DOZ(self):
        ccw1 = Rot3DOZ.getRot3DOZ(alpha=np.pi/2)
        self.assertTrue (ccw1 is not None)
        ccw2 = Rot3DOZ.getRot3DOZ(m=ccw1)
        self.assertTrue (ccw2 is not None)

        r = R.from_euler ('xyz', [0, 0, 90], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))
        self.assertTrue (np.allclose (ccw2, m))

        ccw1.update(alpha=np.pi/4)
        r = R.from_euler ('xyz', [0, 0, 45], degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))

    # TODO : CW
    def testRot3D (self):
        ccw1 = Rot3D.getRot3D (ox=np.pi/4, oy=np.pi/4, oz=np.pi/4)
        self.assertTrue (ccw1 is not None)
        ccw2 = Rot3D.getRot3D (ex=ccw1)
        self.assertTrue (ccw2 is not None)

        r1 = R.from_euler ('x', 45, degrees=True)
        r2 = R.from_euler ('y', 45, degrees=True)
        r3 = R.from_euler ('z', 45, degrees=True)

        r = r1 * r2 * r3
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))
        self.assertTrue (np.allclose (ccw2, m))

        ccw1.update(ox=np.pi/4, oy=0, oz=0)
        r = R.from_euler ('x', 45, degrees=True)
        m = r.as_matrix()

        self.assertTrue (np.allclose (ccw1, m))

    # TODO : both CW & CCw
    def testRot2D (self):
        pass

if __name__ == '__main__':
    unittest.main()

