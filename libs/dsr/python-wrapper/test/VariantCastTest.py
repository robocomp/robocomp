import unittest
from pydsr import *

class Test(unittest.TestCase):

    def test_create_attribute(self):

        a = Node(12, "world", "elmundo")

        #int
        a.attrs["vehicle_id"] = Attribute(10, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["vehicle_id"] = Attribute(10.0, 1)
            a.attrs["vehicle_id"] = Attribute("10.0", 1)
            a.attrs["vehicle_id"] = Attribute(True, 1)
            a.attrs["vehicle_id"] = Attribute([10], 1)
            a.attrs["vehicle_id"] = Attribute([10.0], 1)
            a.attrs["vehicle_id"] = Attribute([10, 12,22], 1)
            a.attrs["vehicle_id"] = Attribute([10.0, 11.2], 1)
            a.attrs["vehicle_id"] = Attribute(22222222222222222222222222222222222222222222222, 1)

        #bool
        a.attrs["vehicle_occupancy"] = Attribute(True, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["vehicle_occupancy"] = Attribute(10.0, 1)
            a.attrs["vehicle_occupancy"] = Attribute("10.0", 1)
            a.attrs["vehicle_occupancy"] = Attribute(1, 1)
            a.attrs["vehicle_occupancy"] = Attribute(0, 1)
            a.attrs["vehicle_occupancy"] = Attribute(1110, 1)
            a.attrs["vehicle_occupancy"] = Attribute([10], 1)
            a.attrs["vehicle_occupancy"] = Attribute([10.0], 1)
            a.attrs["vehicle_occupancy"] = Attribute([10, 12,22], 1)
            a.attrs["vehicle_occupancy"] = Attribute([10.0, 11.2], 1)
            a.attrs["vehicle_occupancy"] = Attribute(22222222222222222222222222222222222222222222222, 1)

        #float
        a.attrs["vehicle_steer"] = Attribute(1.0, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["vehicle_steer"] = Attribute(True, 1)
            a.attrs["vehicle_steer"] = Attribute("10.0", 1)
            a.attrs["vehicle_steer"] = Attribute(1, 1)
            a.attrs["vehicle_steer"] = Attribute(0, 1)
            a.attrs["vehicle_steer"] = Attribute(1110, 1)
            a.attrs["vehicle_steer"] = Attribute([10], 1)
            a.attrs["vehicle_steer"] = Attribute([10.0], 1)
            a.attrs["vehicle_steer"] = Attribute([10, 12,22], 1)
            a.attrs["vehicle_steer"] = Attribute([10.0, 11.2], 1)
            a.attrs["vehicle_steer"] = Attribute(22222222222222222222222222222222222222222222222, 1)

        #double
        a.attrs["test_double_type"] = Attribute(1.0, 1)
        a.attrs["test_double_type"] = Attribute(1.132432543543645646703, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["test_double_type"] = Attribute(True, 1)
            a.attrs["test_double_type"] = Attribute("10.0", 1)
            a.attrs["test_double_type"] = Attribute(1, 1)
            a.attrs["test_double_type"] = Attribute(0, 1)
            a.attrs["test_double_type"] = Attribute(1110, 1)
            a.attrs["test_double_type"] = Attribute([10], 1)
            a.attrs["test_double_type"] = Attribute([10.0], 1)
            a.attrs["test_double_type"] = Attribute([10, 12,22], 1)
            a.attrs["test_double_type"] = Attribute([10.0, 11.2], 1)
            a.attrs["test_double_type"] = Attribute(22222222222222222222222222222222222222222222222, 1)


        #string
        a.attrs["name"] = Attribute("Prueba", 1)
        with self.assertRaises(RuntimeError):
            a.attrs["name"] = Attribute(True, 1)
            a.attrs["name"] = Attribute(10.0, 1)
            a.attrs["name"] = Attribute(1, 1)
            a.attrs["name"] = Attribute(0, 1)
            a.attrs["name"] = Attribute(1110, 1)
            a.attrs["name"] = Attribute([10], 1)
            a.attrs["name"] = Attribute([10.0], 1)
            a.attrs["name"] = Attribute([10, 12,22], 1)
            a.attrs["name"] = Attribute([10.0, 11.2], 1)
            a.attrs["name"] = Attribute(22222222222222222222222222222222222222222222222, 1)


        #vec float
        import numpy as np
        a.attrs["rt_translation"] = Attribute([10.0], 1)
        a.attrs["rt_translation"] = Attribute([10.0, 11.2], 1)
        a.attrs["rt_translation"] = Attribute(np.array([1.2, 4.4], dtype=np.float), 1)
        with self.assertRaises(RuntimeError):
            a.attrs["rt_translation"] = Attribute(True, 1)
            a.attrs["rt_translation"] = Attribute(10.0, 1)
            a.attrs["rt_translation"] = Attribute(1, 1)
            a.attrs["rt_translation"] = Attribute(0, 1)
            a.attrs["rt_translation"] = Attribute(1110, 1)
            a.attrs["rt_translation"] = Attribute([10], 1)
            a.attrs["rt_translation"] = Attribute("[10.0]", 1)
            a.attrs["rt_translation"] = Attribute([10, 12,22], 1)
            a.attrs["rt_translation"] = Attribute(np.array([12, 44], dtype=np.uint8), 1)
            a.attrs["rt_translation"] = Attribute(22222222222222222222222222222222222222222222222, 1)

        #vec bytes
        a.attrs["cam_image"] = Attribute([100], 1)
        a.attrs["cam_image"] = Attribute([100, 112], 1)
        a.attrs["cam_image"] = Attribute(np.array([12, 44], dtype=np.uint8), 1)
        with self.assertRaises(RuntimeError):
            a.attrs["cam_image"] = Attribute(True, 1)
            a.attrs["cam_image"] = Attribute(10.0, 1)
            a.attrs["cam_image"] = Attribute(1, 1)
            a.attrs["cam_image"] = Attribute(0, 1)
            a.attrs["cam_image"] = Attribute(1110, 1)
            a.attrs["cam_image"] = Attribute([10, "a"], 1)
            a.attrs["cam_image"] = Attribute("[10.0]", 1)
            a.attrs["cam_image"] = Attribute([10.0, 12.,22], 1)
            a.attrs["cam_image"] = Attribute(np.array([12.5, 44.2, 1], dtype=np.float), 1)
            a.attrs["cam_image"] = Attribute(22222222222222222222222222222222222222222222222, 1)

        #uint32
        a.attrs["test_uint32_type"] = Attribute(100, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["test_uint32_type"] = Attribute(True, 1)
            a.attrs["test_uint32_type"] = Attribute(10.0, 1)
            a.attrs["test_uint32_type"] = Attribute(1, 1)
            a.attrs["test_uint32_type"] = Attribute(0, 1)
            a.attrs["test_uint32_type"] = Attribute(1110, 1)
            a.attrs["test_uint32_type"] = Attribute([10, "a"], 1)
            a.attrs["test_uint32_type"] = Attribute("[10.0]", 1)
            a.attrs["test_uint32_type"] = Attribute([10.0, 12.,22], 1)
            a.attrs["test_uint32_type"] = Attribute([10, 11], 1)
            a.attrs["test_uint32_type"] = Attribute(np.array([12.5, 44.2, 1], dtype=np.float), 1)
            a.attrs["test_uint32_type"] = Attribute(22222222222222222222222222222222222222222222222, 1)
            a.attrs["test_uint32_type"] = Attribute(-12, 1)


        #uint64
        a.attrs["parent"] = Attribute(100, 1)
        a.attrs["parent"] = Attribute(3689348814741910323, 1)
        with self.assertRaises(RuntimeError):
            a.attrs["parent"] = Attribute(True, 1)
            a.attrs["parent"] = Attribute(10.0, 1)
            a.attrs["parent"] = Attribute(1, 1)
            a.attrs["parent"] = Attribute(0, 1)
            a.attrs["parent"] = Attribute(1110, 1)
            a.attrs["parent"] = Attribute([10, "a"], 1)
            a.attrs["parent"] = Attribute("[10.0]", 1)
            a.attrs["parent"] = Attribute([10.0, 12.,22], 1)
            a.attrs["parent"] = Attribute([10, 11], 1)
            a.attrs["parent"] = Attribute(np.array([12.5, 44.2, 1], dtype=np.float), 1)
            a.attrs["parent"] = Attribute(-12, 1)


        print(a)
        print(a.attrs["cam_image"].value)


if __name__ == '__main__':
    #import psutil
    #if psutil.Process(os.getpid()).parent().name() == 'sh':
    #    unittest.main()
    #else:
    #    print("You probably want to execute test with the runTest.sh script.")
    unittest.main()