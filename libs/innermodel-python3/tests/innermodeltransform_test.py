import unittest
import sys
import numpy as np

sys.path.append('../innermodel_python3')

from innermodeltransform import InnerModelTransform
from innermodelrtmatrix import InnerModelRTMat

class TestInnerModelTransform (unittest.TestCase):
    def test_constructor (self):
        imt = InnerModelTransform ('root', 'static', 1, 1, 1, np.pi/2, 0, 0)
        self.assertTrue (imt is not None)
        self.assertTrue (imt.parent is None)
        self.assertTrue (imt.id == 'root')
        rtmat = InnerModelRTMat.getInnerModelRTMat (1, 1, 1, np.pi/2, 0, 0)
        self.assertTrue (np.allclose (imt.rtmat, rtmat))

    def test_update (self):
        imt = InnerModelTransform ('root', 'static', 1, 1, 1, np.pi/2, 0, 0)
        imt.updatePointers (tx=2, ty=3, tz=4, rx=np.pi/4, ry=0, rz=np.pi/2)
        self.assertTrue (imt.tx == 2 and imt.ty == 3 and imt.tz == 4 and \
                         imt.rx == np.pi/4 and imt.ry == 0 and imt.rz == np.pi/2)
        self.assertTrue (imt.fixed is False)
        imt.updateTranslationPointers (tx=10, ty=40, tz=100)
        self.assertTrue (imt.tx == 10 and imt.ty == 40 and imt.tz == 100)

    # copynode can't be tested without innermodel
    def test_copynode (self):
        pass

if __name__ == '__main__':
    unittest.main()
