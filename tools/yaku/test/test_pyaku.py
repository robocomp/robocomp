import unittest
import context
import yaku_lib


class test_functions(unittest.TestCase):
    def test_get_command_return(self):
        command = 'echo "catpat is black"'
        self.assertEqual(
            (b"catpat is black\n", b""), yaku_lib.get_command_return(command)
        )

    # TODO: Think in a way to test it
    # def test_get_last_child(self):
    #     self.assertEqual(0, yaku_lib.get_last_child(0))


class TestKonsoleSession(unittest.TestCase):
    def setUp(self) -> None:
        self.konsole_session = yaku_lib.KonsoleSession()


class TestYakuakeDBus(unittest.TestCase):
    # def setUp(self):
    #     self.func = YakuakeDBus()

    def test_bus_objects(self):
        self.assertIsNotNone(yaku_lib.YakuakeDBus().sessions)
        self.assertIsNotNone(yaku_lib.YakuakeDBus().tabs)

    def test_bus_singleton(self):
        self.assertEqual(yaku_lib.YakuakeDBus(), yaku_lib.YakuakeDBus())

    def test_session_method(self):
        self.assertIsNotNone(yaku_lib.YakuakeDBus().session(0))


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


if __name__ == "__main__":
    unittest.main()
