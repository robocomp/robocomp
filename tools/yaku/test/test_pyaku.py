import unittest

from pyaku.yaku_lib import YakuakeDBus


class TestYakuakeDBus(unittest.TestCase):
    # def setUp(self):
    #     self.func = YakuakeDBus()

    def test_bus_objects(self):
        self.assertIsNotNone(YakuakeDBus().sessions)
        self.assertIsNotNone(YakuakeDBus().tabs)

    def test_bus_singleton(self):
        self.assertEqual(YakuakeDBus(), YakuakeDBus)

    def test_session_method(self):
        self.assertIsNotNone(YakuakeDBus().session(0))


class Yaku(unittest.TestCase):
    pass
    # def setUp(self):
    #     self.func = YakuakeDBus()

    # def test_bus_objects(self):
    #     self.assertIsNotNone(YakuakeDBus().sessions)
    #     self.assertIsNotNone(YakuakeDBus().tabs)
    #
    # def test_bus_singleton(self):
    #     self.assertEqual(YakuakeDBus(), YakuakeDBus)



if __name__ == '__main__':
    unittest.main()
