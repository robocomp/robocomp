import unittest
import sys

sys.path.append('../innermodel_python3')

from innermodelvector import InnerModelVector

class TestVectorMethods(unittest.TestCase):

    def test_3d(self):
        a = InnerModelVector.vec3d(1., 2., 3.)
        b = InnerModelVector.vec3d(3., 2., 1.)
        assert(a[0] + b[0] == 1. + 3.)
        assert(a[1] + b[1] == 2. + 2.)
        assert(a[2] + b[2] == 3. + 1.)

    def test_6d(self):
        a = InnerModelVector.vec6d(1., 2., 3., 4., 5., 6.)

        assert(a[0] == 1)
        assert(a[5] == 6)
        assert(a.shape[0] == 6)
        assert(a.min() == [1, 0])
        assert(a.max() == [6, 5])

        b = InnerModelVector.vec6d(2, 0, 0, 0, 0, 0)

        assert(a.dot_product(b) == 2)
        assert(b.norm2() == 2)


if __name__ == '__main__':
    unittest.main()
